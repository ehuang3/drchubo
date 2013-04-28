#include "ZmpDriver.h"
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iterator>
#define DEBUG
#define MODULE_NAME "ZMP"
#include "ZmpDebug.h"

using namespace std;
namespace zmp {

ZmpDriver::ZmpDriver()
	: zmp_walker(0) {
}

void ZmpDriver::loop() {
	vector<string> code;
	while(get_code(code)) {
		do_cmd(code);
	}
}

void ZmpDriver::driver_usage(ostream& ostr) {
	ostr << "usage: >> [CMD] [PARAMS]\n"
			"CMD:\n"
			"  print [cmd|gains] [:file]            print X to terminal or file\n"
			"  fill  [cmd|gains] [file]            fill X with file\n"
			"  run                                  run cmd\n"
			"  help  [:cmd|:gains]                  display this text\n"
			"\n"
			"EXTRA:"
			"  Autocompletes cmds, e.g. p c --> print cmd\n"
			"  (does not work on files)\n"
			"\n";
}

istream& ZmpDriver::get_code(vector<string> &code) {
	cout << ">> ";
	string line;
	getline(cin, line);
	DEBUG_STREAM << "Command: " << line << endl;
	istringstream iss(line);
	code.clear();
	copy(istream_iterator<string>(iss),
		 istream_iterator<string>(),
		 back_inserter<vector<string> >(code));
	return cin;
}

void ZmpDriver::do_cmd(vector<string> code) {
	if(code.size() == 0) {
		//cerr << "\33[0;31m" << "invalid string!" << "\33[0m" << endl;
		driver_usage(std::cerr);
		return;
	}
	char cmds[][CMD_LEN] = { "print", "fill", "run", "help" };
	switch (best_match(code[0], cmds, 4)) {
	case 0: do_print(code); break;
	case 1: do_fill(code); break;
	case 2: do_run(code); break;
	case 3: do_help(code); break;
	default: driver_usage(std::cerr); break;
	}
}

void ZmpDriver::do_print(vector<string> code) {
	if(code.size() < 2) {
		driver_usage(std::cerr);
		return;
	}
	char args[][CMD_LEN] = { "cmd", "gains" };
	switch (best_match(code[1], args, 2)) {
	case 0:
		if(code.size() == 2) {
			zcmd.print_command(cout);
		} else {
			DEBUG_STREAM << "Printing to " << code[2] << endl;
			ofstream ofs(code[2].c_str());
			zcmd.print_command(ofs);
			ofs.close();
		}
		break;
	case 1: ERROR_PRINT("Gains not implemented\n"); break;
	default: driver_usage(std::cerr); break;
	}
}

void ZmpDriver::do_run(vector<string> code) {
	if(zmp_walker)
		zmp_walker(zcmd);
	else
		ERROR_STREAM << "zmp_walker = 0 no function to run" << endl;
}

void ZmpDriver::do_fill(vector<string> code) {
	if(code.size() != 3) {
		driver_usage(std::cerr);
		return;
	}
	char args[][CMD_LEN] = { "cmd", "gains" };
	switch (best_match(code[1], args, 2)) {
	case 0: zcmd.fill(code[2].c_str()); break;
	case 1: ERROR_PRINT("Gains not implemented\n"); break;
	default: driver_usage(std::cerr); break;
	}
}

void ZmpDriver::do_help(vector<string> code) {
	if(code.size() == 1) {
		driver_usage(cout);
		return;
	}
	char args[][CMD_LEN] = { "cmd", "gains" };
	switch(best_match(code[1], args, 2)) {
	case 0: zcmd.usage(cout); break;
	case 1: /* TODO: gains */ break;
	default: driver_usage(cerr); break;
	}
}

int ZmpDriver::best_match(std::string word, char cmds[][CMD_LEN], int cmd_size) {
	int max=0;
	int best=-1;
	for(int i=0; i < cmd_size; i++) {
		int m = match(word, cmds[i]);
		if(m > max) {
			best = i;
			max = m;
		}
	}
	return best;
}

int ZmpDriver::match(std::string word, char *cmd) {
	int i = 0;
	while(word.c_str()[i] != 0 && cmd[i] != 0 &&
		  word.c_str()[i] == cmd[i]) {
		i++;
	}
	return i;
}

}
