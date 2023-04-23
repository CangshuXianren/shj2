// $Id: Array.cc 201 2008-05-18 19:47:38Z digasper $
// This file is part of QuadProg++:  
// Copyright (C) 2006--2009 Luca Di Gaspero. 
//
// This software may be modified and distributed under the terms
// of the MIT license.  See the LICENSE file for details.

#include "quadprog/Array.h"

/**
  Index utilities
 */

namespace quadprogpp {

std::set<unsigned int> seq(unsigned int s, unsigned int e)
{
	std::set<unsigned int> tmp;
	for (unsigned int i = s; i <= e; i++)
		tmp.insert(i);
	
	return tmp;
}

std::set<unsigned int> singleton(unsigned int i)
{
	std::set<unsigned int> tmp;
	tmp.insert(i);
	
	return tmp;
}

Matrix<double> cross(const Matrix<double>& rhs1, const Matrix<double>& rhs2)
{
	Matrix<double> tmp(1, 1);
	int rhs1_n = rhs1.nrows(),
		rhs1_m = rhs1.ncols(),
		rhs2_n = rhs2.nrows(),
		rhs2_m = rhs2.ncols();

	if(rhs1_m != rhs2_n){
		std::cout<<"Error in cross:rhs1.m != rhs2.n!"<<std::endl;
		tmp[0][0] = -1;
		return tmp;
	}
	tmp.resize(0, rhs1_n, rhs2_m);
	int cross_num = rhs1_m;
	for(int i=0; i<rhs1_n; i++)
		for(int j=0; j<rhs2_m; j++)
			for(int p=0; p<cross_num; p++)
				tmp[i][j] += rhs1[i][p] * rhs2[p][j];
	return tmp;
}

Matrix<double> power(const Matrix<double>& rhs, const int& a)
{
	Matrix<double> tmp(1, 1);
	int rhs_n = rhs.nrows(),
		rhs_m = rhs.ncols();
	if(0 == a){
		Matrix<double> Tmp(MType::DIAG,1,0,rhs_n,rhs_m);
		return Tmp;
	}else if(1 == a){
		tmp = rhs;
		return tmp;
	}else{
		tmp = rhs;
		for(int i=1; i<a; i++)
			tmp = cross(tmp, rhs);
		return tmp;
	}
}

Matrix<double> kron(const Matrix<double>& rhs1, const Matrix<double>& rhs2)
{
	int rhs1_n = rhs1.nrows(),
		rhs1_m = rhs1.ncols(),
		rhs2_n = rhs2.nrows(),
		rhs2_m = rhs2.ncols();

	Matrix<double> tmp(rhs1_n*rhs2_n, rhs1_m*rhs2_m);
	for(int i=0; i<rhs1_n; i++){
		for(int j=0; j<rhs1_m; j++){
			Matrix<double> tmp1;
			tmp1 = rhs2;
			tmp1 *= rhs1[i][j];
			tmp.setblock(i*rhs2_n, j*rhs2_m, tmp1);
		}
	}
	return tmp;
}

}  // namespace quadprogpp
