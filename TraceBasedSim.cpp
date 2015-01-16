/*********************************************************************************
*  Copyright (c) 2010-2011, Elliott Cooper-Balis
*                             Paul Rosenfeld
*                             Bruce Jacob
*                             University of Maryland 
*                             dramninjas [at] gmail [dot] com
*  All rights reserved.
*  
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions are met:
*  
*     * Redistributions of source code must retain the above copyright notice,
*        this list of conditions and the following disclaimer.
*  
*     * Redistributions in binary form must reproduce the above copyright notice,
*        this list of conditions and the following disclaimer in the documentation
*        and/or other materials provided with the distribution.
*  
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
*  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
*  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
*  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
*  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
*  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
*  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
*  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*********************************************************************************/



//TraceBasedSim.cpp
//
//File to run a trace-based simulation
//

#include <iostream>
#include <fstream>
#include <sstream>
#include <getopt.h>
#include <map>
#include <list>

#include "SystemConfiguration.h"
#include "MemorySystem.h"
#include "MultiChannelMemorySystem.h"
#include "Transaction.h"
#include "IniReader.h"
#include "CSVWriter.h"
#include "util.h"

enum TraceType
{
	k6,
	mase,
	misc
};

using namespace DRAMSim;
using namespace std;

#define RETURN_TRANSACTIONS 1

#ifndef _SIM_
int SHOW_SIM_OUTPUT = 1;
ofstream visDataOut; //mostly used in MemoryController

const uint64_t MAX_PENDING = 1024;
const uint64_t MIN_PENDING = 1023;
uint64_t complete = 0;
uint64_t pending = 0;
uint64_t throttle_count = 0;
uint64_t throttle_cycles = 0;
uint64_t final_cycles = 0;
uint64_t speedup_factor = 0;

// The maximum transaction count for this simuation
uint64_t START_TRANS = 0;
uint64_t MAX_TRANS = 0;

// The cycle counter is used to keep track of what cycle we are on.
uint64_t trace_cycles = 0;

uint64_t last_clock = 0;
uint64_t CLOCK_DELAY = 1000000;

#ifdef RETURN_TRANSACTIONS
class TransactionReceiver
{
	private: 
		map<uint64_t, list<uint64_t> > pendingReadRequests; 
		map<uint64_t, list<uint64_t> > pendingWriteRequests; 
		unsigned numReads, numWrites; 
	
	public: 
		TransactionReceiver() : numReads(0), numWrites(0) 
		{


		}
		void add_pending(const Transaction &t, uint64_t cycle)
		{
			// C++ lists are ordered, so the list will always push to the back and
			// remove at the front to ensure ordering
			if (t.transactionType == DATA_READ)
			{
				pendingReadRequests[t.address].push_back(cycle); 
			}
			else if (t.transactionType == DATA_WRITE)
			{
				pendingWriteRequests[t.address].push_back(cycle); 
			}
			else
			{
				ERROR("This should never happen"); 
				exit(-1);
			}
		}

		void read_complete(unsigned id, uint64_t address, uint64_t done_cycle)
		{
			map<uint64_t, list<uint64_t> >::iterator it;
			it = pendingReadRequests.find(address); 
			if (it == pendingReadRequests.end())
			{
				ERROR("Cant find a pending read for this one"); 
				exit(-1);
			}
			else
			{
				if (it->second.size() == 0)
				{
					ERROR("Nothing here, either"); 
					exit(-1); 
				}
			}

			uint64_t added_cycle = pendingReadRequests[address].front();
			uint64_t latency = done_cycle - added_cycle;

			pendingReadRequests[address].pop_front();
			numReads++;
			complete++;
			pending--;
			if ((complete % 1000 == 0) || (done_cycle - last_clock > CLOCK_DELAY))
			{
				cout << "complete= " << complete << "\t\tpending= " << pending << "\t\t cycle_count= "<< done_cycle << "\t\tthrottle_count=" << throttle_count << "\n";
				last_clock = done_cycle;
			}
			//cout << "Read Callback: #"<<numReads<<": 0x"<< std::hex << address << std::dec << " latency="<<latency<<"cycles ("<< done_cycle<< "->"<<added_cycle<<")"<<endl;
		}

		void write_complete(unsigned id, uint64_t address, uint64_t done_cycle)
		{
			map<uint64_t, list<uint64_t> >::iterator it;
			it = pendingWriteRequests.find(address); 
			if (it == pendingWriteRequests.end())
			{
				ERROR("Cant find a pending read for this one"); 
				exit(-1);
			}
			else
			{
				if (it->second.size() == 0)
				{
					ERROR("Nothing here, either"); 
					exit(-1); 
				}
			}

			uint64_t added_cycle = pendingWriteRequests[address].front();
			uint64_t latency = done_cycle - added_cycle;

			pendingWriteRequests[address].pop_front();
			numWrites++;
			complete++;
			pending--;
			if ((complete % 1000 == 0) || (done_cycle - last_clock > CLOCK_DELAY))
			{
				cout << "complete= " << complete << "\t\tpending= " << pending << "\t\t cycle_count= "<< done_cycle << "\t\tthrottle_count=" << throttle_count << "\n";
				last_clock = done_cycle;
			}
			//cout << "Write Callback: #"<<numWrites<<" 0x"<< std::hex << address << std::dec << " latency="<<latency<<"cycles ("<< done_cycle<< "->"<<added_cycle<<")"<<endl;
		}
};
#endif

void usage()
{
	cout << "DRAMSim2 Usage: " << endl;
	cout << "DRAMSim -t tracefile -s system.ini -d ini/device.ini [-c #] [-p pwd] [-q] [-S 2048] [-n] [-o OPTION_A=1234,tRC=14,tFAW=19]" <<endl;
	cout << "\t-t, --tracefile=FILENAME \tspecify a tracefile to run  "<<endl;
	cout << "\t-s, --systemini=FILENAME \tspecify an ini file that describes the memory system parameters  "<<endl;
	cout << "\t-d, --deviceini=FILENAME \tspecify an ini file that describes the device-level parameters"<<endl;
	cout << "\t-c, --numcycles=# \t\tspecify number of cycles to run the simulation for [default=30] "<<endl;
	cout << "\t-q, --quiet \t\t\tflag to suppress simulation output (except final stats) [default=no]"<<endl;
	cout << "\t-o, --option=OPTION_A=234,tFAW=14\t\t\toverwrite any ini file option from the command line"<<endl;
	cout << "\t-p, --pwd=DIRECTORY\t\tSet the working directory (i.e. usually DRAMSim directory where ini/ and results/ are)"<<endl;
	cout << "\t-S, --size=# \t\t\tSize of the memory system in megabytes [default=2048M]"<<endl;
	cout << "\t-n, --notiming \t\t\tDo not use the clock cycle information in the trace file"<<endl;
	cout << "\t-v, --visfile \t\t\tVis output filename"<<endl;
}
#endif

void *parseTraceFileLine(string &line, uint64_t &addr, enum TransactionType &transType, uint64_t &clockCycle, TraceType type, bool useClockCycle)
{
	size_t previousIndex=0;
	size_t spaceIndex=0;
	uint64_t *dataBuffer = NULL;
	string addressStr="", cmdStr="", dataStr="", ccStr="";

	switch (type)
	{
	case k6:
	{
		spaceIndex = line.find_first_of(" ", 0);

		addressStr = line.substr(0, spaceIndex);
		previousIndex = spaceIndex;

		spaceIndex = line.find_first_not_of(" ", previousIndex);
		cmdStr = line.substr(spaceIndex, line.find_first_of(" ", spaceIndex) - spaceIndex);
		previousIndex = line.find_first_of(" ", spaceIndex);

		spaceIndex = line.find_first_not_of(" ", previousIndex);
		ccStr = line.substr(spaceIndex, line.find_first_of(" ", spaceIndex) - spaceIndex);

		if (cmdStr.compare("P_MEM_WR")==0 ||
		        cmdStr.compare("BOFF")==0)
		{
			transType = DATA_WRITE;
		}
		else if (cmdStr.compare("P_FETCH")==0 ||
		         cmdStr.compare("P_MEM_RD")==0 ||
		         cmdStr.compare("P_LOCK_RD")==0 ||
		         cmdStr.compare("P_LOCK_WR")==0)
		{
			transType = DATA_READ;
		}
		else
		{
			ERROR("== Unknown Command : "<<cmdStr);
			exit(0);
		}

		istringstream a(addressStr.substr(2));//gets rid of 0x
		a>>hex>>addr;

		//if this is set to false, clockCycle will remain at 0, and every line read from the trace
		//  will be allowed to be issued
		if (useClockCycle)
		{
			istringstream b(ccStr);
			b>>clockCycle;
		}
		break;
	}
	case mase:
	{
		spaceIndex = line.find_first_of(" ", 0);

		addressStr = line.substr(0, spaceIndex);
		previousIndex = spaceIndex;

		spaceIndex = line.find_first_not_of(" ", previousIndex);
		cmdStr = line.substr(spaceIndex, line.find_first_of(" ", spaceIndex) - spaceIndex);
		previousIndex = line.find_first_of(" ", spaceIndex);

		spaceIndex = line.find_first_not_of(" ", previousIndex);
		ccStr = line.substr(spaceIndex, line.find_first_of(" ", spaceIndex) - spaceIndex);

		if (cmdStr.compare("IFETCH")==0||
		        cmdStr.compare("READ")==0)
		{
			transType = DATA_READ;
		}
		else if (cmdStr.compare("WRITE")==0)
		{
			transType = DATA_WRITE;
		}
		else
		{
			ERROR("== Unknown command in tracefile : "<<cmdStr);
		}

		istringstream a(addressStr.substr(2));//gets rid of 0x
		a>>hex>>addr;

		//if this is set to false, clockCycle will remain at 0, and every line read from the trace
		//  will be allowed to be issued
		if (useClockCycle)
		{
			istringstream b(ccStr);
			b>>clockCycle;
		}

		break;
	}
	case misc:
		spaceIndex = line.find_first_of(" ", spaceIndex+1);
		if (spaceIndex == string::npos)
		{
			ERROR("Malformed line: '"<< line <<"'");
		}

		addressStr = line.substr(previousIndex,spaceIndex);
		previousIndex=spaceIndex;

		spaceIndex = line.find_first_of(" ", spaceIndex+1);
		if (spaceIndex == string::npos)
		{
			cmdStr = line.substr(previousIndex+1);
		}
		else
		{
			cmdStr = line.substr(previousIndex+1,spaceIndex-previousIndex-1);
			dataStr = line.substr(spaceIndex+1);
		}

		//convert address string -> number
		istringstream b(addressStr.substr(2)); //substr(2) chops off 0x characters
		b >>hex>> addr;

		// parse command
		if (cmdStr.compare("read") == 0)
		{
			transType=DATA_READ;
		}
		else if (cmdStr.compare("write") == 0)
		{
			transType=DATA_WRITE;
		}
		else
		{
			ERROR("INVALID COMMAND '"<<cmdStr<<"'");
			exit(-1);
		}
		if (SHOW_SIM_OUTPUT)
		{
			DEBUGN("ADDR='"<<hex<<addr<<dec<<"',CMD='"<<transType<<"'");//',DATA='"<<dataBuffer[0]<<"'");
		}

		//parse data
		//if we are running in a no storage mode, don't allocate space, just return NULL
#ifndef NO_STORAGE
		if (dataStr.size() > 0 && transType == DATA_WRITE)
		{
			// 32 bytes of data per transaction
			dataBuffer = (uint64_t *)calloc(sizeof(uint64_t),4);
			size_t strlen = dataStr.size();
			for (int i=0; i < 4; i++)
			{
				size_t startIndex = i*16;
				if (startIndex > strlen)
				{
					break;
				}
				size_t charsLeft = min(((size_t)16), strlen - startIndex + 1);
				string piece = dataStr.substr(i*16,charsLeft);
				istringstream iss(piece);
				iss >> hex >> dataBuffer[i];
			}
			PRINTN("\tDATA=");
			BusPacket::printData(dataBuffer);
		}

		PRINT("");
#endif
		break;
	}
	return dataBuffer;
}

#ifndef _SIM_

void alignTransactionAddress(Transaction &trans)
{
	Config &cfg = trans.cfg; 
	// zero out the low order bits which correspond to the size of a transaction

	unsigned throwAwayBits = dramsim_log2((cfg.BL*cfg.JEDEC_DATA_BUS_BITS/8));

	trans.address >>= throwAwayBits;
	trans.address <<= throwAwayBits;
}

/** 
 * Override options can be specified on the command line as -o key1=value1,key2=value2
 * this method should parse the key-value pairs and put them into a map 
 **/ 
OptionsMap parseParamOverrides(const string &kv_str)
{
	OptionsMap kv_map;
	size_t start = 0, comma=0, equal_sign=0;
	// split the commas if they are there
	while (1)
	{
		equal_sign = kv_str.find('=', start); 
		if (equal_sign == string::npos)
		{
			break;
		}

		comma = kv_str.find(',', equal_sign);
		if (comma == string::npos)
		{
			comma = kv_str.length();
		}

		string key = kv_str.substr(start, equal_sign-start);
		string value = kv_str.substr(equal_sign+1, comma-equal_sign-1); 

		kv_map[key] = value; 
		start = comma+1;

	}
	return kv_map; 
}

void old_TBS(string traceFileName, string systemIniFilename, string deviceIniFilename, string pwdString, 
	     string visFilename, unsigned megsOfMemory, bool useClockCycle, OptionsMap paramOverrides, unsigned numCycles)
{
	 

	// get the trace filename
	string temp = traceFileName.substr(traceFileName.find_last_of("/")+1);

	TraceType traceType;
	//get the prefix of the trace name
	temp = temp.substr(0,temp.find_first_of("_"));
	if (temp=="mase")
	{
		traceType = mase;
	}
	else if (temp=="k6")
	{
		traceType = k6;
	}
	else if (temp=="misc")
	{
		traceType = misc;
	}
	else
	{
		ERROR("== Unknown Tracefile Type : "<<temp);
		exit(0);
	}


	// no default value for the default model name
	if (deviceIniFilename.length() == 0)
	{
		ERROR("Please provide a device ini file");
		usage();
		exit(-1);
	}


	//ignore the pwd argument if the argument is an absolute path
	if (pwdString.length() > 0 && traceFileName[0] != '/')
	{
		traceFileName = pwdString + "/" +traceFileName;
	}

	DEBUG("== Loading trace file '"<<traceFileName<<"' == ");

	ifstream traceFile;
	string line;


	CSVWriter &CSVOut = CSVWriter::GetCSVWriterInstance(visFilename); 
	MultiChannelMemorySystem *memorySystem = new MultiChannelMemorySystem(deviceIniFilename, systemIniFilename, pwdString, traceFileName, megsOfMemory, CSVOut, &paramOverrides);
	// set the frequency ratio to 1:1
	memorySystem->setCPUClockSpeed(0);
	Config &cfg = memorySystem->cfg;


#ifdef RETURN_TRANSACTIONS
	TransactionReceiver transactionReceiver; 
	/* create and register our callback functions */
	Callback_t *read_cb = new Callback<TransactionReceiver, void, unsigned, uint64_t, uint64_t>(&transactionReceiver, &TransactionReceiver::read_complete);
	Callback_t *write_cb = new Callback<TransactionReceiver, void, unsigned, uint64_t, uint64_t>(&transactionReceiver, &TransactionReceiver::write_complete);
	memorySystem->registerCallbacks(read_cb, write_cb, NULL);
#endif


	uint64_t addr;
	uint64_t clockCycle=0;
	enum TransactionType transType;

	void *data = NULL;
	int lineNumber = 0;
	Transaction *trans=NULL;
	bool pendingTrans = false;

	traceFile.open(traceFileName.c_str());

	if (!traceFile.is_open())
	{
		cout << "== Error - Could not open trace file"<<endl;
		exit(0);
	}
	for (size_t i=0;i<numCycles;i++)
	{
		if (!pendingTrans)
		{
			if (!traceFile.eof())
			{
				getline(traceFile, line);

				if (line.size() > 0)
				{
					data = parseTraceFileLine(line, addr, transType,clockCycle, traceType,useClockCycle);
					trans = new Transaction(transType, addr, data, cfg);
					alignTransactionAddress(*trans); 

					if (i>=clockCycle)
					{
						if (!memorySystem->addTransaction(trans))
						{
							pendingTrans = true;
						}
						else
						{
#ifdef RETURN_TRANSACTIONS
							transactionReceiver.add_pending(*trans, i); 
#endif
							// the memory system accepted our request so now it takes ownership of it
							trans = NULL; 
						}
					}
					else
					{
						pendingTrans = true;
					}
				}
				else
				{
					DEBUG("WARNING: Skipping line "<<lineNumber<< " ('" << line << "') in tracefile");
				}
				lineNumber++;
			}
			else
			{
				//we're out of trace, set pending=false and let the thing spin without adding transactions
				pendingTrans = false; 
			}
		}

		else if (pendingTrans && i >= clockCycle)
		{
			pendingTrans = !memorySystem->addTransaction(trans);
			if (!pendingTrans)
			{
#ifdef RETURN_TRANSACTIONS
				transactionReceiver.add_pending(*trans, i); 
#endif
				trans=NULL;
			}
		}
		memorySystem->update();
	}

	traceFile.close();
	memorySystem->printStats(true);
	// make valgrind happy
	if (trans)
	{
		delete trans;
	}
	delete(memorySystem);
}

void simple_TBS(string traceFileName, string systemIniFilename, string deviceIniFilename, string pwdString, 
		string visFilename, unsigned megsOfMemory, bool useClockCycle, OptionsMap paramOverrides)
{
	// no default value for the default model name
	if (deviceIniFilename.length() == 0)
	{
		ERROR("Please provide a device ini file");
		usage();
		exit(-1);
	}

	DEBUG("== Loading trace file '"<<traceFileName<<"' == ");

	CSVWriter &CSVOut = CSVWriter::GetCSVWriterInstance(visFilename); 
	MultiChannelMemorySystem *memorySystem = new MultiChannelMemorySystem(deviceIniFilename, systemIniFilename, pwdString, traceFileName, megsOfMemory, CSVOut, &paramOverrides);
	// set the frequency ratio to 1:1
	memorySystem->setCPUClockSpeed(0); 
	Config &cfg = memorySystem->cfg;


#ifdef RETURN_TRANSACTIONS
	TransactionReceiver transactionReceiver; 
	/* create and register our callback functions */
	Callback_t *read_cb = new Callback<TransactionReceiver, void, unsigned, uint64_t, uint64_t>(&transactionReceiver, &TransactionReceiver::read_complete);
	Callback_t *write_cb = new Callback<TransactionReceiver, void, unsigned, uint64_t, uint64_t>(&transactionReceiver, &TransactionReceiver::write_complete);
	memorySystem->registerCallbacks(read_cb, write_cb, NULL);
#endif

	void *data = NULL;
	Transaction *trans=NULL;
	enum TransactionType transType;
	// Open input file
	ifstream inFile;
	inFile.open(traceFileName.c_str(), ifstream::in);
	if (!inFile.is_open())
	{
		cout << "ERROR: Failed to load tracefile: " << traceFileName << "\n";
		abort();
	}
	

	char char_line[256];
	string line;
	bool done = false;
	uint64_t trans_count = 0;

	// if we're fast forwarding some
	if(START_TRANS != 0)
	{
		while(inFile.good() && !done)
		{
			inFile.getline(char_line, 256);
			trans_count++;
			if(trans_count >= START_TRANS)
			{
				done = true;
			}
		}		
	}
	done = false;
	bool paused = false;
	bool write = 0;

	while (inFile.good() && !done)
	{
		if(!paused)
		{
			// Read the next line.
			inFile.getline(char_line, 256);
			line = (string)char_line;

			// Filter comments out.
			size_t pos = line.find("#");
			line = line.substr(0, pos);
			
			// Strip whitespace from the ends.
			line = strip(line);
			
			// Filter newlines out.
			if (line.empty())
				continue;
			
			// Split and parse.
			list<string> split_line = split(line);
			
			if (split_line.size() != 3)
			{
				cout << "ERROR: Parsing trace failed on line:\n" << line << "\n";
				cout << "There should be exactly three numbers per line\n";
				cout << "There are " << split_line.size() << endl;
				abort();
			}
			
			uint64_t line_vals[3];
			
			int i = 0;
			for (list<string>::iterator it = split_line.begin(); it != split_line.end(); it++, i++)
			{
				// convert string to integer
				uint64_t tmp;
				convert_uint64_t(tmp, (*it));
				line_vals[i] = tmp;
			}

			// Finish parsing.
			uint64_t trans_cycle;
			if(speedup_factor != 0)
			{
				trans_cycle = line_vals[0] / speedup_factor;
			}
			else
			{
				trans_cycle = line_vals[0];
			}
			write = line_vals[1] % 2;
			if(write == 0)
				transType = DATA_READ;
			else
				transType = DATA_WRITE;
			uint64_t addr = line_vals[2];

			// increment the counter until >= the clock cycle of cur transaction
			// for each cycle, call the update() function.
			while (trace_cycles < trans_cycle)
			{
				memorySystem->update();
				trace_cycles++;
			}
			trans = new Transaction(transType, addr, data, cfg);
			alignTransactionAddress(*trans); 
		}

		// add the transaction and continue
		if(memorySystem->addTransaction(trans))
		{		
			//memorySystem->addTransaction(write, addr);
#ifdef RETURN_TRANSACTIONS
				transactionReceiver.add_pending(*trans, trace_cycles); 
#endif
			pending++;
			trans_count++;
			paused = false;
		}
		else
		{
			paused = true;
			memorySystem->update();
			throttle_cycles++;
		}
			
			// If the pending count goes above MAX_PENDING, wait until it goes back below MIN_PENDING before adding more 
			// transactions. This throttling will prevent the memory system from getting overloaded.
		if (pending >= MAX_PENDING)
		{
			//cout << "MAX_PENDING REACHED! Throttling the trace until pending is back below MIN_PENDING.\t\tcycle= " << trace_cycles << "\n";
			throttle_count++;
			while (pending > MIN_PENDING)
			{
				memorySystem->update();
				throttle_cycles++;
			}
			//cout << "Back to MIN_PENDING. Allowing transactions to be added again.\t\tcycle= " << trace_cycles << "\n";
		}

		// check to see if we're done with the trace for now
		if(MAX_TRANS != 0 && trans_count >= MAX_TRANS)
			done = true;
	}

	inFile.close();


	//mem->syncAll();


	// Run update until all transactions come back.
	while (pending > 0)
	{
		memorySystem->update();
		final_cycles++;
	}

	// This is a hack for the moment to ensure that a final write completes.
	// In the future, we need two callbacks to fix this.
	// This is not counted towards the cycle counts for the run though.
	//for (int i=0; i<1000000; i++)
	//	memorySystem->update();

	cout << "trace_cycles = " << trace_cycles << "\n";
	cout << "throttle_count = " << throttle_count << "\n";
	cout << "throttle_cycles = " << throttle_cycles << "\n";
	cout << "final_cycles = " << final_cycles << "\n";
	cout << "total_cycles = trace_cycles + throttle_cycles + final_cycles = " << trace_cycles + throttle_cycles + final_cycles << "\n\n";
	
	memorySystem->printStats(true);
}

int main(int argc, char **argv)
{
	int c;
	string traceFileName;
	string systemIniFilename("system.ini");
	string deviceIniFilename;
	string pwdString;
	string visFilename("dramsim.vis");
	unsigned megsOfMemory=2048;
	bool useClockCycle=true;
	unsigned numCycles=1000;
	OptionsMap paramOverrides; 

	bool KISS = false;

	stringstream ss;
	//getopt stuff
	while (1)
	{
		static struct option long_options[] =
		{
			{"deviceini", required_argument, 0, 'd'},
			{"tracefile", required_argument, 0, 't'},
			{"systemini", required_argument, 0, 's'},

			{"pwd", required_argument, 0, 'p'},
			{"numcycles",  required_argument,	0, 'c'},
			{"option",  required_argument,	0, 'o'},
			{"quiet",  no_argument, &SHOW_SIM_OUTPUT, 'q'},
			{"help", no_argument, 0, 'h'},
			{"size", required_argument, 0, 'S'},
			{"visfile", required_argument, 0, 'v'},
			{"keep_simple", no_argument, 0, 'k'},
			{"end_trans", required_argument, 0, 'e'},
			{0, 0, 0, 0}
		};
		int option_index=0; //for getopt
		c = getopt_long (argc, argv, "t:s:c:d:o:p:S:v:e:qkn", long_options, &option_index);
		if (c == -1)
		{
			break;
		}
		switch (c)
		{
		case 0: //TODO: figure out what the hell this does, cuz it never seems to get called
			if (long_options[option_index].flag != 0) //do nothing on a flag
			{
				printf("setting flag\n");
				break;
			}
			printf("option %s",long_options[option_index].name);
			if (optarg)
			{
				printf(" with arg %s", optarg);
			}
			printf("\n");
			break;
		case 'h':
			usage();
			exit(0);
			break;
		case 't':
			traceFileName = string(optarg);
			break;
		case 's':
			systemIniFilename = string(optarg);
			break;
		case 'd':
			deviceIniFilename = string(optarg);
			break;
		case 'c':
			numCycles = atoi(optarg);
			break;
		case 'S':
			megsOfMemory=atoi(optarg);
			break;
		case 'p':
			pwdString = string(optarg);
			break;
		case 'q':
			SHOW_SIM_OUTPUT=false;
			break;
		case 'n':
			useClockCycle=false;
			break;
		case 'o':
			paramOverrides = parseParamOverrides(string(optarg)); 
			break;
		case 'v':
			visFilename = string(optarg);
			break;
		case 'k':
			KISS = true;
			break;
		case 'e':
			ss << optarg;
			ss >> MAX_TRANS;
			ss.clear();
			break;
		case '?':
			usage();
			exit(-1);
			break;
		}
	}

	if(KISS)
	{
		simple_TBS(traceFileName, systemIniFilename, deviceIniFilename, pwdString, visFilename, megsOfMemory, useClockCycle, paramOverrides);
	}
	else
	{
		old_TBS(traceFileName, systemIniFilename, deviceIniFilename, pwdString, visFilename, megsOfMemory, useClockCycle, paramOverrides, numCycles);
	}
}
#endif
