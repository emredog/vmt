#pragma once

#include <string>
#include <vector>
#include <iostream>

using namespace std;

class HelperFunctions
{
private: 
	HelperFunctions(void);
	~HelperFunctions(void);
public:
	//function to get entries in a directory
	static void getDir(string& d, vector<string> & f);
	//function to calculate sign of a value
	static double sgn(int val) {
		return (0.0 < (double)val) - ((double)val < 0.0);
	}
};

