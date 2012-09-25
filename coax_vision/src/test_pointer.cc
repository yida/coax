#include <iostream>
#include <vector>

using namespace std;

typedef unsigned char uint8_t;

int main(int argc, char **argv) {
	uint8_t *data = new uint8_t[30];
    uint8_t *char_ptr = data;
    for (unsigned int i = 0; i < 30; i++) {
		*(data+i) = i;
	}	    
	//cout << sizeof(unsigned char) << endl;
    for (unsigned int j = 0; j < 30; j+=5) {
		cout << j << endl;
	}
//	delete data;
	return 0;
}
