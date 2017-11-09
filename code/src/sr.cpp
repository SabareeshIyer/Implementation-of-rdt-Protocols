#include "../include/simulator.h"
#include <iostream>
#include <unistd.h>
#include <deque>
#include <queue>
#include <string>
#include <vector>
#include <algorithm>

#define data_size 20
#define A 0
#define B 1
#define EVENT_TIMEOUT 0
#define EVENT_ACK_ARRIVED 1
#define TIMER_TIMEOUT 30
#define NOT_ACKED 0
#define ACKED 1

using namespace std;

/* ******************************************************************
 ALTERNATING BIT AND GO-BACK-N NETWORK EMULATOR: VERSION 1.1  J.F.Kurose

   This code should be used for PA2, unidirectional data transfer 
   protocols (from A to B). Network properties:
   - one way network delay averages five time units (longer if there
     are other messages in the channel for GBN), but can be larger
   - packets can be corrupted (either the header or the data portion)
     or lost, according to user-defined probabilities
   - packets will be delivered in the order in which they were sent
     (although some can be lost).
**********************************************************************/

/********* STUDENTS WRITE THE NEXT SEVEN ROUTINES *********/

int create_checksum(char data[20], int seqnum, int acknum = 0);
pkt sender_make_pkt (int seqnum, char data[20], int checksum);
pkt receiver_make_pkt(int expectedseqnum_r);
void buffer_msg(char data[20]);
void rcvr_buffer_msg(struct pkt packet);
void print_queue(std::queue<string> q);
void print_queue_r(std::queue<pkt> q);
void store_time(int seqnum, float time);
int next_unacked();
int unacked_after_this(int seqnum);
int next_unreceived();
void deliver_buffer_data();
void deliver_remaining_buffer_data(int s);
void deliver_redone_buffer_data(int seqnum);
int next_buffer_data();
int lowest_time_unacked_packet(int s);

struct timer_pair{
	int seqnum;
	float time;
};

deque<pkt> sndpkt;
deque<pkt> rcvpkt;
std::queue<string> buffer;
std::queue<pkt> rcvr_buffer;
std::vector<pkt> redone_rcvr_buffer(1000);
std::vector<int> ack_status(1);
std::vector<int> rcv_status(1);
std::vector<int> delivery_status(1000, 0);
vector<timer_pair> timer_array(1000);
int window_size;
int send_base;
int rcv_base = 0;
int nextseqnum_s = 1;
int expectedseqnum_r;
int timer_packet_seq;
int buffer_base = -1;
int buffer_gap;
bool timer_running = false;
int last_delivered_seq = -1;

/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message)
{
	char* data_to_send;
	data_to_send = message.data;	

	if(nextseqnum_s < send_base+window_size){
		int checksum = create_checksum(data_to_send, nextseqnum_s);
		sndpkt.push_back(sender_make_pkt(nextseqnum_s, data_to_send, checksum));

		tolayer3(A, sndpkt[nextseqnum_s-1]);
		ack_status.push_back(NOT_ACKED);
		cout<<"Sending packet with seqnum "<<nextseqnum_s<<"\tData contained is "<<data_to_send<<endl;
		if(send_base == nextseqnum_s || timer_running == false){
			starttimer(A, TIMER_TIMEOUT);
			timer_running = true;
			timer_packet_seq = nextseqnum_s;
			cout<<"Timer is now associated to packet "<<timer_packet_seq<<endl;
		}
		else{
			cout<<"Not starting timer because send_base is "<<send_base<<" and nextseqnum_s is "<<nextseqnum_s<<endl;
		}
		store_time(nextseqnum_s, get_sim_time());
		nextseqnum_s++;
	}	
	else{
		buffer_msg(data_to_send);
		//refuse data
	}

}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet)
{
	char data[20];
	int check_checksum;
	for(int i=0; i<data_size; i++)
		data[i] = packet.payload[i];
	check_checksum = create_checksum(data, packet.seqnum, packet.acknum);
	cout<<"ACK received with acknum = "<< packet.acknum<<endl;
	if(packet.acknum > nextseqnum_s)
		return;

	//checking if not corrupt
	if(check_checksum == packet.checksum){
		cout<<"Checksum checks out. Marking packet as ACKED.\n";
		if(!(packet.acknum <= send_base+window_size && packet.acknum >= send_base))
			return;
		ack_status[packet.acknum] = ACKED;
		//if(packet.acknum == timer_packet_seq)
			//stoptimer(A);
		int index_first_not_acked = next_unacked();
		cout<<"Index of ack of "<<packet.acknum<<" is "<<ack_status[packet.acknum]<<endl;
		
		//no unACKed packets
		if(index_first_not_acked == -1){
			stoptimer(A);
			timer_running = false;
			send_base++;
			cout<<"send_base updated to "<<send_base<<" (by increment)\n";
		}

		if(index_first_not_acked > packet.acknum || index_first_not_acked > send_base){
			send_base = index_first_not_acked;
			cout<<"send_base updated to "<<send_base<<" (by index_first_not_acked)\n";
		}

		if(send_base == nextseqnum_s && index_first_not_acked != -1){
			stoptimer(A);
			timer_running = false;
		}

		while(!buffer.empty() && nextseqnum_s < send_base+window_size){	
			string str = buffer.front();
			buffer.pop();
			char* data_to_send = const_cast<char*>(str.c_str());
			int checksum = create_checksum(data_to_send, nextseqnum_s);
			sndpkt.push_back(sender_make_pkt(nextseqnum_s, data_to_send, checksum));

			tolayer3(A, sndpkt[nextseqnum_s-1]);
			ack_status.push_back(NOT_ACKED);
			cout<<"Sending packet with seqnum "<<nextseqnum_s<<"\tData contained is "<<data_to_send<<endl;
			if(send_base == nextseqnum_s){
				starttimer(A, TIMER_TIMEOUT);
				timer_running = true;
				timer_packet_seq = nextseqnum_s;
				cout<<"Timer is now associated to packet "<<timer_packet_seq<<endl;
			}
			else{
				cout<<"Not starting timer because send_base is "<<send_base<<" and nextseqnum_s is "<<nextseqnum_s<<endl;
			}
			store_time(nextseqnum_s, get_sim_time());
			nextseqnum_s++;		
		}
	}

}

/* called when A's timer goes off */
void A_timerinterrupt()
{
	
	if(ack_status[timer_packet_seq] == 1){
		timer_packet_seq = send_base;
		//return;
	}


	cout<<"Timer interrupt. Resending packet with seqnum "<<timer_packet_seq<<"\n";
	tolayer3(A, sndpkt[timer_packet_seq-1]);
	store_time(timer_packet_seq, get_sim_time());
	//cout<<"Timer array has: ";
	//for(int i = 1; i< timer_array.size()/sizeof(timer_pair); i++)
	//	cout<<timer_array[i].time<<" ";
	cout<<endl;
	int next_unacked_seq = lowest_time_unacked_packet(timer_packet_seq); //unacked_after_this(timer_packet_seq);
	if(next_unacked_seq == -1){
		starttimer(A, TIMER_TIMEOUT);
		timer_running = true;
		cout<<"this Timer started with timeout "<<TIMER_TIMEOUT;
		return;
	}
	int timeout = timer_array[next_unacked_seq].time + TIMER_TIMEOUT - get_sim_time();
	starttimer(A, timeout);
	timer_running = true;
	cout<<"that Timer started with timeout "<<timeout;
	timer_packet_seq = next_unacked_seq;
	cout<<"Timer is now associated to packet "<<timer_packet_seq<<endl;
}  

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{
	window_size = getwinsize();
	nextseqnum_s = 1;
	send_base = 1;
	ack_status[0] = 2;
	for(int i = 0; i < 1000; i++){
		timer_array[i].seqnum = -1;
		timer_array[i].time = -1;
	}


}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
	//if(rcv*_base < next_unreceived())
		//rcv_base = next_unreceived();

	char data[20];
	int check_checksum;
	for(int i=0; i<data_size; i++)
		data[i] = packet.payload[i];
	check_checksum = create_checksum(data, packet.seqnum, packet.acknum);
	cout<<"Data ("<<data<<") has been received at B.\npacket seqnum = "<<packet.seqnum<<" expected is "<< expectedseqnum_r<<endl;

	if(check_checksum != packet.checksum){
		cout<<"Corrupted packet. Not responding to this.\n";
		return;
	}
	//after this, packet cannot be corrupt
	
	cout<<"Size of rcv_status is "<<rcv_status.size()<<endl;
	for(int i = rcv_status.size(); i < packet.seqnum; i++){
		rcv_status.push_back(0);
		cout<<"rcv_status ["<<i<<"] = "<<rcv_status.back()<<"\n";
	}
	if(packet.seqnum >= rcv_status.size())
		rcv_status.push_back(1);
	else
		rcv_status[packet.seqnum] = 1;

	if(rcv_status[packet.seqnum] == 0)
		rcv_status[packet.seqnum] = 1;

	cout<<"rcv_status array holds: \n";
	for(int i= 1; i< rcv_status.size(); i++)
		cout<<rcv_status[i]<<" ";
	cout<<endl;

	cout<<"checking if packet is in window...";
	cout<<" packet seqnum is "<<packet.seqnum<<", rcv_base is "<<rcv_base<<endl;

	if(packet.seqnum >= rcv_base && packet.seqnum < rcv_base+window_size){
		cout<<"Packet is in window.\n";
		rcvpkt.push_back(receiver_make_pkt(packet.seqnum));	//create ACK packet
		tolayer3(B, rcvpkt.back());							//sending ACK for packet
		bool in_order = false;
		int nu = next_unreceived();
		//if(nu == -1)
		//	in_order = true;

		if(packet.seqnum < expectedseqnum_r && delivery_status[packet.seqnum] == 0){
			//tolayer5(B, data);
			//last_delivered_seq = packet.seqnum;
			//delivery_status[packet.seqnum] = 1;
		}

		if(nu > packet.seqnum || packet.seqnum == expectedseqnum_r)
			in_order = true;

		if(in_order == false && delivery_status[packet.seqnum] == 0){
			cout<<"Buffering out of order packet.\n";
			if(buffer_base == -1)
				buffer_base = packet.seqnum;
			rcvr_buffer_msg(packet);
		}

		if(in_order == true){
			expectedseqnum_r++;
			cout<<"Packet received is in order.\n";
			//deliver all buffered/in-order packets 
			/*
			while(!rcvr_buffer.empty()){
				while(rcvr_buffer.front().seqnum < packet.seqnum){
					deliver_buffer_data();
				}
				break;
			}
			*/
			int b = buffer_base;
			for(int i = last_delivered_seq; i < packet.seqnum; i++){
				cout<<"i is "<<i<<endl;
				if(i == -1)
					break;
				deliver_redone_buffer_data(i);

			}

			
			if(delivery_status[packet.seqnum] == 0){
				cout<<"Delivering data that was just received.\nData delivered is: "<<data<<endl;
				tolayer5(B, data);
				last_delivered_seq = packet.seqnum;
				cout<<"Data delivered because status of "<<packet.seqnum<<" is "<<delivery_status[packet.seqnum]<<endl;
				delivery_status[packet.seqnum] = 1;
			}
			else{
				cout<<"This packet has already been delivered.\n";
			}
			
			rcv_base = packet.seqnum+1;
			cout<<"Updated rcv_base to "<<rcv_base<<endl;
			int check_seq = packet.seqnum;
			cout<<"Buffer_base is "<<buffer_base<<" and packet.seqnum+1 is "<<packet.seqnum+1<<endl;

			if(buffer_base >= packet.seqnum + 1 && delivery_status[packet.seqnum+1] == 0){
				deliver_remaining_buffer_data(packet.seqnum+1);
			}

			cout<<"Updating expectedseqnum_r to "<<expectedseqnum_r<<endl;	

			/*
	inga:	int seq_diff = 0;
			if(!rcvr_buffer.empty())
				seq_diff = rcvr_buffer.front().seqnum - check_seq;
			if(seq_diff == 1){
				check_seq = rcvr_buffer.front().seqnum;
				cout<<"Value of check_seq is "<<check_seq<<endl;
				deliver_buffer_data();
				cout<<"Going to inga.\n";
				if(expectedseqnum_r < check_seq){
					expectedseqnum_r = check_seq+1;
					cout<<"check_seq of "<<rcvr_buffer.front().payload<<" is "<<check_seq<<endl;
					cout<<"Updating expectedseqnum_r to "<<expectedseqnum_r<<endl;
				}
				goto inga;
			}
			*/
			//expectedseqnum_r = check_seq+1;
			

			//advance rcv_base to next packet yet to be received (not next expectedseqnum_r)
			//if(nu != -1){
			//rcv_base = nu;
			//cout<<"rcv_base set to nu.\n";
			//}
			//else{
			//	rcv_base++;
			//	cout<<"rcv_base incremented to "<<rcv_base<<endl;
			//}
		}

	}

	else if(packet.seqnum >= rcv_base-window_size && packet.seqnum <rcv_base){
		rcvpkt.push_back(receiver_make_pkt(packet.seqnum));	//create ACK packet
		tolayer3(B, rcvpkt.back());							//sending ACK for packet
	}
	else{
		cout<<"Packet is in neither window.\n";
		//do nothing
	}

	if(packet.seqnum == expectedseqnum_r)
		expectedseqnum_r++;

}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
	expectedseqnum_r = 1;
	rcv_status[0] = 2;
	delivery_status[0] = 2;
	rcv_base=1;
	cout<<"Size of rcv_status is "<<rcv_status.size()<<endl;

	struct pkt p;
	p.seqnum = -1;
	p.acknum = -1;
	p.checksum = -1;
	for(int i=0;i<1000; i++){
		redone_rcvr_buffer[i] = p;
	}
}


pkt sender_make_pkt (int seqnum, char data[20], int checksum){
	struct pkt packet_to_send;
	for(int i  = 0; i<data_size;i++)
		packet_to_send.payload[i] = data[i];
	packet_to_send.seqnum = seqnum;
	//preliminary implementation
	packet_to_send.acknum = 0;
	packet_to_send.checksum = checksum;
	return packet_to_send;	
}

pkt receiver_make_pkt(int expectedseqnum_r){
	struct pkt packet;
	packet.seqnum = 0;
	packet.acknum = expectedseqnum_r;
	for(int i=0;i<data_size;i++)
		packet.payload[i] = '_';
	packet.checksum = create_checksum(packet.payload, packet.seqnum, packet.acknum);
	return packet;
}

int create_checksum(char data[20], int seqnum, int acknum){
	int checksum = 0+seqnum+acknum;
	for(int i = 0;i<data_size; i++)
		checksum = checksum + data[i];
	return checksum;
}

void buffer_msg(char data[20]){
	string str(data,20);
	buffer.push(str);
	cout<<"Added to buffer. Buffer now is:\n";
	print_queue(buffer);
}

void rcvr_buffer_msg(struct pkt packet){
	
	//rcvr_buffer.push(packet);

	redone_rcvr_buffer[packet.seqnum] = packet;


	cout<<"Buffer now contains: \n";
	//print_queue_r(rcvr_buffer);
	cout<<"Not going to print.\n";

	/*
	string str(data,20);
	rcvr_buffer.push(str);
	cout<<"Added to receiver buffer. Buffer now is:\n";
	print_queue(rcvr_buffer);
	*/
}

void print_queue(std::queue<string> q){
  while (!q.empty()){
    std::cout << q.front() << " ";
    q.pop();
  }
  std::cout << std::endl;
}

void print_queue_r(std::queue<pkt> q){
	/*
  while (!q.empty()){
    std::cout << q.front().payload << " ";
    q.pop();
  }
  std::cout << std::endl;
  */
	/*
	int i = buffer_base;
	while(redone_rcvr_buffer[i].seqnum != -1){
		for(int j =0; j < data_size; j++)
			data[j] = redone_rcvr_buffer[i].payload[j];
		cout<<data<<" ";
		i++;	
	}
	cout<<endl;
	*/
}

void store_time(int seqnum, float time){
	cout<<"Storing time for packet "<<seqnum<<endl;
	/*
	if(seqnum > timer_array.size()/sizeof(timer_pair)){
		timer_pair t;
		t.seqnum = seqnum;
		t.time = time;
		timer_array.push_back(t);
	}
	else{*/
	timer_array[seqnum].time = time;
	cout<<"Time at which packet "<<seqnum<<"has been sent is "<<time<<endl;
	//}
	return;
}

int next_unacked(){
	int index_first_not_acked = -1;
	for(int i=0; i<ack_status.size();i++){
		if(ack_status[i] == NOT_ACKED){
			index_first_not_acked = i;
			break;
		}
	}
	cout<<"Index first not acked is "<<index_first_not_acked<<"\n";
	return index_first_not_acked;
}

int unacked_after_this(int seqnum){
	int index_unacked_after_this = -1;
	for(int i=seqnum+1; i<ack_status.size();i++){
		if(ack_status[i] == NOT_ACKED){	
			index_unacked_after_this = i;					
		}
	}
	cout<<"Index unacked after this is is "<<index_unacked_after_this<<"\n";
	return index_unacked_after_this;
}

int lowest_time_unacked_packet(int s){
	int retval = -1;
	std::vector<timer_pair> v;
	for(int i = 1; i<ack_status.size();i++){
		if(ack_status[i] == NOT_ACKED && i != s)
			v.push_back(timer_array[i]);
	}

	cout<<"Number of unacked packets (v-size) is "<<v.size()/sizeof(timer_pair)<<endl;

	float min_t = 55000;
	int min_t_seq = 1111;
	for(int i = 0; i<v.size()/sizeof(timer_pair); i++){
		if(v[i].time < min_t){
			min_t = v[i].time;
			min_t_seq = v[i].seqnum;
		}
	}
	cout<<"min_t is "<<min_t<<" and min_t_seq is "<<min_t_seq<<endl;
	if(min_t_seq != 1111)
		retval = min_t_seq;
	return retval;
}

int next_unreceived(){
	int index_first_unrcvd = -1;
	for(int i=0; i<rcv_status.size();i++){
		if(rcv_status[i] == 0){
			index_first_unrcvd = i;
			cout<<"rcv_status["<<i<<"] is "<<0<<endl;
			break;
		}
	}
	//if(index_first_unrcvd == -1)
		//index_first_unrcvd = expectedseqnum_r;
	cout<<"Index first not received is "<<index_first_unrcvd<<endl;
	return index_first_unrcvd;
}

void deliver_buffer_data(){
	cout<<"Delivering data from buffer head.\n";
	struct pkt packet = rcvr_buffer.front();
	rcvr_buffer.pop();
	char data[20];
	for(int i=0; i < data_size; i++)
		data[i] = packet.payload[i];
	cout<<"Retrieved data from rcvr_buffer, it now holds: \n";
	print_queue_r(rcvr_buffer);
	tolayer5(B, data);
	last_delivered_seq = packet.seqnum;
	delivery_status[packet.seqnum] = 1;
	cout<<"Data sent to layer5 from buffer: "<<data<<endl;
}

void deliver_remaining_buffer_data(int s){
	int i = s;
	int temp = -1;
	char data[20];
	while(redone_rcvr_buffer[i].seqnum != -1){
		if(i >= 999)
			return;
		rcv_base = i+1;

		cout<<"Is this happening?\n";
		for(int j =0; j< data_size; j++)
			data[j] = redone_rcvr_buffer[i].payload[j];
		tolayer5(B, data);
		last_delivered_seq = redone_rcvr_buffer[i].seqnum;
		delivery_status[redone_rcvr_buffer[i].seqnum] = 1;
		cout<<"remaining Data sent to layer5 from buffer: "<<data<<endl;
		cout<<"Delivery status of seqnum "<<redone_rcvr_buffer[i].seqnum<<" set to 1\n";
		temp = redone_rcvr_buffer[i].seqnum;
		redone_rcvr_buffer[i].seqnum = -1;
		i++;		
	}
	if(temp != -1)
		expectedseqnum_r = temp+1;

	cout<<"After delivering remaining data, expectedseqnum_r is set to "<<expectedseqnum_r<<endl;
	buffer_base = next_buffer_data();
}

void deliver_redone_buffer_data(int seqnum){
	char data[20];
	for(int i = 0; i < data_size; i++)
		data[i] = redone_rcvr_buffer[seqnum].payload[i];
	if(delivery_status[seqnum] == 0){
		tolayer5(B, data);
		last_delivered_seq = seqnum;
		redone_rcvr_buffer[seqnum].seqnum = -1;
		delivery_status[seqnum] = 1;
		cout<<"redone Data sent to layer5 from buffer: "<<data<<endl;
		buffer_base = next_buffer_data();
	}
	else{
		cout<<"Seqnum "<< seqnum <<" has already been delivered.\n" ;
		buffer_base = next_buffer_data();
	}
}


int next_buffer_data(){
	int retval = -1;
	for(int i=1; i< 1000; i++){
		if(redone_rcvr_buffer[i].seqnum != -1)
			retval = i;
	}
	return retval;
}