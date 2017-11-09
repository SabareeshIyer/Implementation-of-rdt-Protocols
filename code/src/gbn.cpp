#include "../include/simulator.h"
#include <iostream>
#include <unistd.h>
#include <deque>
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

int create_checksum(char data[20], int seqnum, int acknum = 0);
pkt sender_make_pkt (int seqnum, char data[20], int checksum);
pkt receiver_make_pkt(int expectedseqnum_r);
void buffer_msg(char data[20]);
void print_queue(std::queue<string> q);

deque<pkt> sndpkt;
struct pkt rcvpkt;
std::queue<string> buffer;
int window_size;
int base;
int nextseqnum_s = 1;
int expectedseqnum_r;
float sent_time;
float TIMER_TIMEOUT = 20;
float estimatedRTT = -1;
float sampleRTT;

/* called from layer 5, passed the data to be sent to other side */
void A_output(struct msg message)
{
	char* data_to_send;
	data_to_send = message.data;
		
	if(nextseqnum_s < base+window_size){
		int checksum = create_checksum(data_to_send, nextseqnum_s);
		sndpkt.push_back(sender_make_pkt(nextseqnum_s, data_to_send, checksum));

		tolayer3(A, sndpkt[nextseqnum_s-1]);
		cout<<"Sending packet with seqnum "<<nextseqnum_s<<"\tData contained is "<<data_to_send<<endl;
		if(base == nextseqnum_s){
			starttimer(A, TIMER_TIMEOUT);
			sent_time = get_sim_time();
		}
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
	if(check_checksum == packet.checksum){
		base = packet.acknum + 1;

		/************************/
		sampleRTT = get_sim_time() - sent_time;
		if(estimatedRTT == -1)
			estimatedRTT = sampleRTT;
		estimatedRTT = 0.875 * estimatedRTT + 0.125 * sampleRTT;

		TIMER_TIMEOUT = estimatedRTT;
		if(TIMER_TIMEOUT < -20)
			TIMER_TIMEOUT =TIMER_TIMEOUT * -1;
			//failsafes to ensure correctness in case of simulator chronology problem
			if(TIMER_TIMEOUT <= 20)
				TIMER_TIMEOUT = 20;
			cout<<"Timeout set to "<<TIMER_TIMEOUT<<endl;
		/***********************/
		
		if(base == nextseqnum_s)
			stoptimer(A);
		else{
			starttimer(A, TIMER_TIMEOUT);
			sent_time = get_sim_time();
		}
		while(!buffer.empty() && nextseqnum_s < base+window_size){	
			string str = buffer.front();
			buffer.pop();
			char* data_to_send = const_cast<char*>(str.c_str());
			int checksum = create_checksum(data_to_send, nextseqnum_s);
			sndpkt.push_back(sender_make_pkt(nextseqnum_s, data_to_send, checksum));

			tolayer3(A, sndpkt[nextseqnum_s-1]);
			cout<<"Sending packet with seqnum "<<nextseqnum_s<<"\tData contained is "<<data_to_send<<endl;
			if(base == nextseqnum_s){
				starttimer(A, TIMER_TIMEOUT);
				sent_time = get_sim_time();
			}
			nextseqnum_s++;		
		}
	}
}

/* called when A's timer goes off */
void A_timerinterrupt()
{
	starttimer(A, TIMER_TIMEOUT);
	sent_time = get_sim_time();
	cout<<"TIMEOUT\nRetransmitting from packet "<<base<<endl;;
	for(int i = base; i<nextseqnum_s;i++){
		tolayer3(A, sndpkt[i-1]);
		cout<<"Sending packet with seqnum "<<i<<endl;
	}
}  

/* the following routine will be called once (only) before any other */
/* entity A routines are called. You can use it to do any initialization */
void A_init()
{
	window_size = getwinsize();
	nextseqnum_s = 1;
	base = 1;
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
	cout<<"Data ("<<data<<") has been received at B.\npacket seqnum = "<<packet.seqnum<<" expected is "<< expectedseqnum_r<<endl;

	if(packet.seqnum == expectedseqnum_r)
		cout<<"The checksums are "<<check_checksum<<packet.checksum<<endl;
	else{
		cout<<"Expected and received seq nums don't match. Resending ACK of last properly received packet.\nACK num is "<<rcvpkt.acknum<<endl;
		tolayer3(B, rcvpkt);
		return;
	}

	if(check_checksum == packet.checksum && packet.seqnum == expectedseqnum_r){
		cout<<"Data received; sending to layer5 and replying with ACK\n";
		tolayer5(B, data);
		rcvpkt = receiver_make_pkt(expectedseqnum_r);
		tolayer3(B, rcvpkt);
		expectedseqnum_r++;
	}
}

/* the following rouytine will be called once (only) before any other */
/* entity B routines are called. You can use it to do any initialization */
void B_init()
{
	expectedseqnum_r = 1;
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

int create_checksum(char data[20], int seqnum, int acknum){
	int checksum = 0+seqnum+acknum;
	for(int i = 0;i<data_size; i++)
		checksum = checksum + data[i];
	return checksum;
}

pkt receiver_make_pkt(int expectedseqnum_r){
	struct pkt packet;
	packet.seqnum = 0;
	packet.acknum = expectedseqnum_r;
	for(int i=0;i<data_size;i++)
		packet.payload[i] = 'a';
	packet.checksum = create_checksum(packet.payload, packet.seqnum, packet.acknum);
	return packet;
}

void buffer_msg(char data[20]){
	string str(data,20);
	buffer.push(str);
	cout<<"Added to buffer. Buffer now is:\n";
	print_queue(buffer);
}

void print_queue(std::queue<string> q){
  while (!q.empty()){
    std::cout << q.front() << " ";
    q.pop();
  }
  std::cout << std::endl;
}