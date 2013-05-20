#pragma once
#include <string>
#include <vector>
#include <iostream>
#include <Eigen/Dense>

namespace zmp {

class gain_element_t {
public:
	gain_element_t(std::string _gname, double _val);

	const std::string& name() const { return gname; }
	void name(std::string _name) { gname = _name; }

	double val() const { return gval; }
	double& ref_val() { return gval; }
	void val(double _val) { gval = _val; }

	bool regex_match(std::string _regex) const;

	friend std::ostream& operator<<(std::ostream &out, const gain_element_t& gain);
	friend std::istream& operator>>(std::istream &in, gain_element_t& gain);

private:
	std::string gname;  // gain identifier
	double gval;		// gain value
};

class gain_list_t {
public:
	gain_list_t();

	void add(const gain_element_t& _gele) { gain_list.push_back(_gele); }
	gain_element_t get_element(int i) const { return gain_list.at(i); }
	int size() const { return gain_list.size(); }

	std::vector<gain_element_t> list() const { return gain_list; }
	void list(std::vector<gain_element_t> _list) { gain_list = _list; }

	void fill(const char *fpath);
	void replace(const char *fpath);
	void save(const char *fpath) const;
	void print(std::ostream& ostr) const;

	// <regex> [=|*|/|+|-] <double>
	void eval_expr(std::string _expr, bool _add);

	// returns gains that match regex
	Eigen::VectorXd match(std::string regex) const;
	// returns all gains in a vector
	Eigen::VectorXd to_vector() const;

private:
	std::vector<gain_element_t> gain_list;

};

}
