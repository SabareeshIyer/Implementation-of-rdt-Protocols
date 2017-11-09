
/*
Command to run:
./abt -s 10 -w 5 -m 25 -l 0.0 -c 0 .0 -t 50 -v 2 > log.txt
*/
#include "../include/simulator.h"
#include <iostream>
#include <unistd.h>
#include <queue>
#include <string>

#define data_size 20
#define A 0
#define B 1
#define EVENT_TIMEOUT 0
#define EVENT_ACK_ARRIVED 1
//#define TIMER_TIMEOUT 20

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

void fsm_abt_sender(char data[20]);
void fsm_abt_s_state_1 (char data[20]);
void fsm_abt_s_state_2 (int state);
void fsm_abt_s_state_3 (char data[20]);
void fsm_abt_s_state_4 (int state);
void buffer_msg(char data[20]);
//char* get_buffer_head();
void send_next_in_buffer();
int create_checksum(char data[20], int seqnum, int acknum = 0);
pkt sender_make_pkt (int seqnum, char data[20], int checksum);
void print_queue(std::queue<string> q);

struct pkt sndpkt;
struct pkt rcvpkt;
std::queue<string> buffer;
int buffer_size = 0;
int awaiting_ack = 0;
int counter = 0;
int now_accepting_seq = 0;
int in_flight = 0;
int prev_seq = 1;
int packets_received = 0;
float sent_time;
float TIMER_TIMEOUT = 20;
float sampleRTT = 20;

/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message)
{
	char* data_to_send;
	data_to_send = message.data;
//	cout<<"Message arrived from layer 5. Contents: "<<data_to_send<<"\n";
	if(buffer_size != 0){
		cout<<"Adding message from layer 5 to buffer.\n";
		buffer_msg(data_to_send);
		if(in_flight != 0){
			//data_to_send = get_buffer_head();
			return;
		}
	}
	fsm_abt_sender(data_to_send);	
}

/* called from layer 3, when a packet arrives for layer 4 */
void A_input(struct pkt packet)
{
	//currently implemented only for no corruption case
	char data[20];
	int check_checksum;
	for(int i=0; i<data_size; i++)
		data[i] = packet.payload[i];
	check_checksum = create_checksum(data, packet.seqnum, packet.acknum);
	if(check_checksum != packet.checksum){
		cout<<"Received ACK is corrupt. Waiting for timeout - to resend.\n";
		return;
	}

	bool flag = false;
	//add logic to check if the ack of the incoming packet is zero
	if(awaiting_ack == 0 && awaiting_ack == packet.acknum){
		cout<<"ACK arrived for packet 0\n";
		in_flight--;
		flag = true;
		fsm_abt_s_state_2(EVENT_ACK_ARRIVED);
	}
	//add logic to check if the ack of the incoming packet is 1
	if(awaiting_ack == 1 && awaiting_ack == packet.acknum){
		cout<<"ACK arrived for packet 1\n";
		in_flight--;
		flag = true;
		fsm_abt_s_state_4(EVENT_ACK_ARRIVED);
	}
	if(buffer_size != 0 && flag == true)
		send_next_in_buffer();
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
	if(awaiting_ack == 0){
		fsm_abt_s_state_2(EVENT_TIMEOUT);
	}

	if(awaiting_ack == 1){
		fsm_abt_s_state_4(EVENT_TIMEOUT);
	}
}  

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{

}

/* Note that with simplex transfer from a-to-B, there is no B_output() */

/* called from layer 3, when a packet arrives for layer 4 at B*/
void B_input(struct pkt packet)
{
	char data[20];
	int check_checksum;
	for(int i=0; i<data_size; i++)
		data[i] = packet.payload[i];
	check_checksum = create_checksum(data, packet.seqnum, packet.acknum);

	//checking for corruption
	cout<<"Received data is "<<packet.payload<<"\t with seqnum "<<packet.seqnum<<endl;
	cout<<"check_checksum is "<<check_checksum<<" and rcvd checksum is "<<packet.checksum<<endl;

	//cout<<"Prev seq is "
	
	if(check_checksum != packet.checksum){
		if(packets_received == 0)
			return;
		cout<<"Received packet is corrupt. Resending ACK of prev.\n";
		tolayer3(B, rcvpkt);
		return;
	}

	if(packets_received !=0 && (prev_seq == packet.seqnum || packet.seqnum > 1)){
		cout<<"Resending ACK of previous packet...\n";
		tolayer3(B, rcvpkt);
		return;
	}
	
	if(packet.seqnum > 1){
		return;
	}

	prev_seq = packet.seqnum;
	
	tolayer5(B, data);
	packets_received++;

	cout<<"Message arrived with seq "<<packet.seqnum<<"\nsending reply...\n";

	for(int i=0;i<data_size;i++)
		rcvpkt.payload[i] = 'a';
	rcvpkt.seqnum = 0;
	rcvpkt.acknum = packet.seqnum;
	rcvpkt.checksum = create_checksum(rcvpkt.payload, rcvpkt.seqnum, rcvpkt.acknum);
	
	tolayer3(B, rcvpkt);
	
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{

}

void fsm_abt_sender(char data[20]){
	if(in_flight != 0){
		cout<<"There are packets in flight. So buffering data.\n";
		buffer_msg(data);
		return;
	}

	if(counter%2 == 0){
		counter++;
		fsm_abt_s_state_1 (data);
	}
	else{
		counter++;
		fsm_abt_s_state_3 (data);
	}
}

void fsm_abt_s_state_1 (char data[20]){
	if(now_accepting_seq != 0)
		return;
	int seqnum	 = 0;
	int checksum = create_checksum(data, seqnum);
	sndpkt = sender_make_pkt (seqnum,data,checksum);
	tolayer3(A, sndpkt);
	in_flight++;
	cout<<"Message sent with seqnum 0\n";
	starttimer(A, TIMER_TIMEOUT);
	sent_time = get_sim_time();
	awaiting_ack = 0;
	//return;
}

void fsm_abt_s_state_2(int state){
	if(state == EVENT_TIMEOUT){
		tolayer3(A, sndpkt);
		//don't need to update in_flight here^ because this is a retransmission
		stoptimer(A);
		starttimer(A, TIMER_TIMEOUT);
		//restarting timer^
	}

	if(state == EVENT_ACK_ARRIVED){
		stoptimer(A);
		now_accepting_seq = 1;
		TIMER_TIMEOUT = TIMER_TIMEOUT * 0.875 + 0.125 * get_sim_time() - sent_time;
		if(TIMER_TIMEOUT < 15)
			TIMER_TIMEOUT = 20;
	}
	return;
}

void fsm_abt_s_state_3(char data[20]){
	if(now_accepting_seq != 1)
		return;
	int seqnum   = 1;
	int checksum = create_checksum(data, seqnum);
	sndpkt = sender_make_pkt(seqnum, data, checksum);
	tolayer3(A, sndpkt);
	in_flight++;
	cout<<"Message sent with seqnum 1\n";
	starttimer(A, TIMER_TIMEOUT);
	sent_time = get_sim_time();
	awaiting_ack = 1;
	//return;
}

void fsm_abt_s_state_4(int state){
	if(state == EVENT_TIMEOUT){
		tolayer3(A, sndpkt);
		//don't need to update in_flight here^ because this is a retransmission
		stoptimer(A);
		starttimer(A, TIMER_TIMEOUT);
		//restarting timer^
	}

	if(state == EVENT_ACK_ARRIVED){
		stoptimer(A);
		now_accepting_seq = 0;
		TIMER_TIMEOUT = TIMER_TIMEOUT * 0.875 + 0.125 * get_sim_time() - sent_time;
		if(TIMER_TIMEOUT < 15)
			TIMER_TIMEOUT = 20;
	}
	return;
}

pkt sender_make_pkt (int seqnum, char data[20], int checksum){
	struct pkt packet_to_send;
	//preliminary implementation
	packet_to_send.seqnum = seqnum;	
	packet_to_send.acknum = 0;
	packet_to_send.checksum = checksum;
	for(int i  = 0; i<data_size;i++)
		packet_to_send.payload[i] = data[i];	
	return packet_to_send;	
}

void buffer_msg(char data[20]){
	string str(data,20);
	buffer.push(str);
	buffer_size++;
	cout<<"Added to buffer. Buffer now is:\n";
	print_queue(buffer);
}

/*
char* get_buffer_head(){
	string str = buffer.front();
	char* data =  const_cast<char*>(str.c_str());
	buffer_size--;
	buffer.pop();
	cout<<"get_buffer_head called. Removed message from buffer. Buffer now is:\n";
	print_queue(buffer);
	return data;
}
*/

void send_next_in_buffer(){
	string str = buffer.front();
	char* data =  const_cast<char*>(str.c_str());
	buffer_size--;
	buffer.pop();
	cout<<"Removed message from buffer. Buffer now is:\n";
	print_queue(buffer);
	cout<<"Retrieved from buffer. Next to be sent is "<<data<<"\n";
	fsm_abt_sender(data);
}

int create_checksum(char data[20], int seqnum, int acknum){
	int checksum = 0+seqnum+acknum;
	for(int i = 0;i<data_size; i++)
		checksum = checksum + data[i];
	return checksum;
}

void print_queue(std::queue<string> q){
  while (!q.empty()){
    std::cout << q.front() << " ";
    q.pop();
  }
  std::cout << std::endl;
}