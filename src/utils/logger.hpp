#ifndef __LOGGER_HPP__
#define __LOGGER_HPP__

#include <iostream>
#include <iomanip>
#include <fstream>

#include <utils/aux.hpp>

#include <eigen3/Eigen/Dense>

using namespace std;

template<typename Derived>
ofstream & operator<<(ofstream &s, const Eigen::MatrixBase<Derived> &m)
{
	for (int i = 0; i < m.size(); i++) {
		if (i > 0)
			s << ",";
		
		s << m(i, 0);
	}

	return s;
}

class Logger
{
public:
	Logger(string file_name, int prec=4, char sep=',') :
		sep(sep)
	{
		if (file_exists(file_name)) {
			cerr << "Log file " << file_name << " already exists!" << endl;
			exit(EXIT_FAILURE);
		}
		this->file.open(file_name);
		line_start = true;
		this->set_prec(prec);
	}

	~Logger()
	{
		this->file.close();
	}

	void set_prec(int prec) {
		this->file << fixed << showpoint << setprecision(prec);
	}

	void line_start_check()
	{
		if (!this->line_start) {
			this->file << sep;
		}
		this->line_start = false;
	}


	template<typename T>
	friend Logger & operator<<(Logger &logger, const T &obj)
	{
		logger.line_start_check();
		logger.file << obj;

		return logger;
	}

	friend Logger & operator<<(Logger &logger, const char &c)
	{
		if (c == '\n') {
			logger.line_start = true;
		}
		else {
			logger.line_start_check();
		}
		logger.file << c;

		return logger;
	}

	template<typename T>
	friend Logger* operator<<(Logger *logger, const T &obj)
	{
		*(logger) << obj;
		return logger;
	}

	void flush() { this->file.flush(); }
	void close() { this->file.close(); }

private:
	ofstream file;
	char sep;
	bool line_start;
};

#endif