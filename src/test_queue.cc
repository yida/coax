#include <iostream>
#include <queue>

using namespace std;

struct SymAxis {
	unsigned short axis;
	unsigned int value;
};

class CompareSymAxis {
	public:
	bool operator() (SymAxis& A1, SymAxis& A2) {
		if (A1.value > A2.value) 
			return true;
		else 
			return false;
	}
};

int main() {
	priority_queue<SymAxis, vector<SymAxis>, CompareSymAxis> Axis;
	SymAxis A[4] = {{1,3456},{2,66421},{3,1234},{4,111111}};
	for (unsigned int i = 0; i < 4; i++)
		Axis.push(A[i]);
	while (!Axis.empty()) {
		SymAxis axis = Axis.top();
		cout << axis.axis << " " << axis.value << endl;
		Axis.pop();
	}
	return 0;
}
