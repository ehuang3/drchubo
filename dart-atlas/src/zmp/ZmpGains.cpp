#include "ZmpGains.h"
#include <regex>
#include <sstream>
#include <fstream>
#include <iterator>
#define DEBUG
#define MODULE_NAME "ZMP-GAINS"
#include "ZmpDebug.h"

using namespace std;
using namespace Eigen;

namespace zmp {

gain_element_t::gain_element_t(string _gname, double _val)
	: gname(_gname),
	  gval(_val) {
}

bool gain_element_t::regex_match(string _regex) const {
	std::regex e(_regex.c_str());
	return std::regex_match(gname, e);
}

ostream& operator<<(ostream &out, const gain_element_t& gain) {
	return out << gain.name() << " = " << gain.val();
}

istream& operator>>(istream &in, gain_element_t& gain) {
	string name;
	double val;
	in >> name >> val;
	gain.name(name);
	gain.val(val);
	return in;
}

gain_list_t::gain_list_t() {}

void gain_list_t::fill(const char *fpath) {
	ifstream ifs(fpath);
	if(!ifs) {
		ERROR_PRINT("Failed to open file: %s\n", fpath);
		return;
	}
	DEBUG_PRINT("Clearing and repopulating gains list\n");
	gain_list.clear();
	string line;
	while(getline(ifs, line)) {
		eval_expr(line, true);
	}
	return;
}

void gain_list_t::replace(const char *fpath) {
	ifstream ifs(fpath);
	if(!ifs) {
		ERROR_PRINT("Failed to open file: %s\n", fpath);
		return;
	}
	DEBUG_PRINT("Replacing and appending gains list\n");
	string line;
	while(getline(ifs, line)) {
		eval_expr(line, true);
	}
	return;
}

void gain_list_t::save(const char *fpath) const {
	DEBUG_PRINT("Saving to %s\n", fpath);
	ofstream ofs(fpath);
	print(ofs);
	ofs.close();
}

void gain_list_t::print(ostream& ostr) const {
	for(int i=0; i < gain_list.size(); i++) {
		ostr << gain_list[i] << endl;
	}
}

void gain_list_t::eval_expr(string _expr, bool _add) {
	istringstream iss(_expr);
	vector<string> code;
	copy(istream_iterator<string>(iss),
		 istream_iterator<string>(),
		 back_inserter<vector<string> >(code));
	if(code.size() != 3) {
		ERROR_PRINT("Malformed expression: %s\n", _expr.c_str());
		ERROR_PRINT("Try (with spaces): <name|regex> [=|*|/|+|-] <double>\n");
		return;
	}
	char *endptr;
	double operand = strtod(code[2].c_str(), &endptr);
	if (!endptr || *endptr) {
		ERROR_PRINT("Failed to parse number: %s\n", code[2].c_str());
		ERROR_PRINT("Try (with spaces): <name|regex> [=|*|/|+|-] <double>\n");
		return;
	}
	if(code[1].size() != 1) {
		ERROR_PRINT("Failed to recognize operation: %s\n", code[1].c_str());
		ERROR_PRINT("Try (with spaces): <name|regex> [=|*|/|+|-] <double>\n");
		return;
	}
	char op = code[1][0];
	bool match = false;
	for(int i=0; i < gain_list.size(); i++) {
		if(gain_list[i].regex_match(code[0])) {
			gain_element_t old_gain = gain_list[i];
			switch (op) {
			case '=': gain_list[i].ref_val()  = operand; break;
			case '*': gain_list[i].ref_val() *= operand; break;
			case '/': gain_list[i].ref_val() /= operand; break;
			case '-': gain_list[i].ref_val() -= operand; break;
			case '+': gain_list[i].ref_val() += operand; break;
			default:
				ERROR_PRINT("Failed to recognize operation: %c\n", op);
				ERROR_PRINT("Try (with spaces): <name|regex> [=|*|/|+|-] <double>\n");
				return;
			}
			DEBUG_STREAM << old_gain << " --> " << gain_list[i].val() << endl;
			match = true;
		}
	}
	if(!match && op == '=' && _add) {
		DEBUG_PRINT("Adding gain element %s\n", code[0].c_str());
		gain_list.push_back(gain_element_t(code[0], operand));
	} else if(!match) {
		ERROR_PRINT("No match for: %s\n", code[0].c_str());
	}
	return;
}

VectorXd gain_list_t::match(string _regex) const {
	vector<double> vals;
	for(int i=0; i < gain_list.size(); i++) {
		if(gain_list[i].regex_match(_regex)) {
			vals.push_back(gain_list[i].val());
		}
	}
	VectorXd gains(vals.size());
	for(int i=0; i < vals.size(); i++) {
		gains(i) = vals[i];
	}
	return gains;
}

VectorXd gain_list_t::to_vector() const {
	VectorXd gains(gain_list.size());
	for(int i=0; i < gain_list.size(); i++) {
		gains(i) = gain_list[i].val();
	}
	return gains;
}

}
