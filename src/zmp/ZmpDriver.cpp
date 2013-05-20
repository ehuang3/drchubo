#include "ZmpDriver.h"
#include <vector>
#include <string>
#include <sstream>
#include <fstream>
#include <iterator>
#define DEBUG
#define MODULE_NAME "ZMP-DRIVER"
#include "ZmpDebug.h"

using namespace std;
namespace zmp {

ZmpDriver::ZmpDriver()
	: zmp_walker(0),
	  zmp_reset(0) {
}

void ZmpDriver::driver_usage(ostream& ostr) {
	ostr << "ZMP-DRIVER USAGE:\n"
			"  ZMP-CMD\n"
			"    cmd                          Prints ZMP-CMD to terminal\n"
			"    cmd <param> <val>            Sets ZMP-CMD <param> to <val>\n"
			"    cmd  >  <file>               Saves ZMP-CMD to <file>\n"
			"    cmd  <  <file>               Loads ZMP-CMD from <file>\n"
			"\n"
			"  ZMP-GAINS\n"
			"    gains                        Prints ZMP-GAINS to terminal\n"
			"    gains <expr>                 Evaluates <expr> on ZMP-GAINS\n"
			"    gains  >  <file>             Saves ZMP-CMD to <file>\n"
			"    gains  <  <file>             Loads ZMP-CMD from <file>\n"
			"\n"
			"    <expr> := <regex> [=|*|\\|+|-] <double>\n"
			"\n"
			"  USER\n"
			"    run                          Runs user ZMP-WALKER function\n"
			"    reset                        Runs user ZMP-RESET function\n"
			"    debug                        Toggle debug output\n"
			"    help  [:cmd]                 Show help\n"
			"\n"
			"  TIPS\n"
			"    - Longest match on commands, e.g. h c -> help cmd\n"
			"    - Short ZMP-CMD options accepted, e.g. X -> zmp-offset-x\n"
			"\n";
}

void ZmpDriver::loop() {
	vector<string> code;
	string line;
	while(get_code(code, line)) {
		exec_driver(code, line);
	}
}

void ZmpDriver::next_command(ZmpCommand& _cmd, gain_list_t& _gains) {
	vector<string> code;
	string line;
	while(get_code(code, line)) {
		if(code.size() > 0 && code[0][0] == 'r') {
			break;
		}
		exec_driver(code, line);
	}
	_cmd = zcmd;
	_gains = zgains;
	return;
}

istream& ZmpDriver::get_code(vector<string> &code, string &line) {
	cout << ">> ";
	getline(cin, line);
	DEBUG_STREAM << "Received command: " << line << endl;
	istringstream iss(line);
	code.clear();
	copy(istream_iterator<string>(iss),
		 istream_iterator<string>(),
		 back_inserter<vector<string> >(code));
	return cin;
}

void ZmpDriver::exec_driver(vector<string> &code, string &line) {
	if(code.size() == 0) {
		//driver_usage(std::cerr);
		return;
	}
	char cmds[][CMD_LEN] = { "cmd", "gains", "run", "reset", "debug", "help" };
	switch (best_match(code[0], cmds, 6)) {
	case 0: do_cmd(code, line); break;
	case 1: do_gains(code, line); break;
	case 2:
		if(zmp_walker) zmp_walker(zcmd, zgains);
		else ERROR_PRINT("No function ZMP-WALKER to run\n");
		break;
	case 3:
		if(zmp_reset) zmp_reset();
		else ERROR_PRINT("No function ZMP-RESET to run\n");
		break;
	case 4:
		ERROR_PRINT("Not implmented\n");
		break;
	case 5:
		if(code.size() == 2 && code[1][0] == 'c') zcmd.usage(cout);
		else driver_usage(cout);
		break;
	default: driver_usage(std::cerr); break;
	}
}

void ZmpDriver::do_cmd(vector<string> &code, string &line) {
	if(code.size() == 1) {
		zcmd.print_command(cout);
		return;
	}
	if(code.size() != 3) {
		ERROR_PRINT("Malformed command: %s\n", line.c_str());
		return;
	}
	switch (code[1][0]) {
	case '<':
		DEBUG_PRINT("Loading %s into ZMP-CMD\n", code[2].c_str());
		zcmd.fill(code[2].c_str());
		break;
	case '>':
		DEBUG_PRINT("Saving ZMP-CMD into %s\n", code[2].c_str());
		{
		ofstream ofs(code[2].c_str());
		zcmd.print_command(ofs);
		ofs.close();
		}
		break;
	default:
		zcmd.fill(code[1], code[2]);
		break;
	}
}

void ZmpDriver::do_gains(vector<string> &code, string &line) {
	if(code.size() == 1) {
		zgains.print(cout);
		return;
	}
	switch (code[1][0]) {
	case '<':
		DEBUG_PRINT("Loading %s into ZMP-GAINS\n", code[2].c_str());
		zgains.fill(code[2].c_str());
		break;
	case '>':
		DEBUG_PRINT("Saving ZMP-GAINS into %s\n", code[2].c_str());
		zgains.save(code[2].c_str());
		break;
	default:
		string expr;
		for(int i=1; i < code.size(); i++) {
			expr += code[i] + " ";
		}
		zgains.eval_expr(expr, false);
		break;
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
