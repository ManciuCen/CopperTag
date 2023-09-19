#ifndef _AAMED_LINK_MATRIX_H_
#define _AAMED_LINK_MATRIX_H_

#include <iostream>
#include <vector>
#include <cstring>

struct SORTDATA;

bool cmp(SORTDATA &A, SORTDATA &B);

template<class T>
class GroupPart {
public:
	GroupPart()
	{
		_data = NULL;
		data_size = 0;
	}
	GroupPart(int num)
	{
		_data = new T[num];
		data_size = num;
	}

	void Update(int new_num)
	{
		use_size = new_num;
		if (new_num > data_size)
		{
			if (_data != NULL)
				delete[] _data;
			_data = new T[new_num];
			data_size = new_num;
		}
		memset(_data, 0, sizeof(T)*new_num);
	}
	
	void clean()
	{
		memset(_data, 0, sizeof(T)*use_size);
	}
	int memsize() { return data_size; }
	int usesize() { return use_size; }
	T* GetDataPoint() const { return _data; }
	void release()
	{
		if (_data != NULL)
		{
			delete[] _data;
			_data = NULL;
		}
			
	}

	T* operator[](int idx) { return _data + idx; }
protected:
	T *_data;
	int data_size;
	int use_size;
}; // class GroupPart

struct SORTDATA {
	double val;
	int idx_l;
	int idx_r;
}; // struct SORTDATA

#endif // _AAMED_LINK_MATRIX_H_