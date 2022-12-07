#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <sstream>
 
using namespace std;
 
int main()
{
    string fname;
    cout << "Enter the file name: ";
    cin >> fname;
    
    fstream file (fname, ios::in);

    if (!file.is_open()) { 
        cout << "Could not open the file" << endl;
        return 1;
    }

    vector<vector<string>> content;
    string line, word;

    while (getline(file, line)) {
        vector<string> row;
        stringstream str(line);
        
        while (getline(str, word, ',')) { row.push_back(word); }
        content.push_back(row);
    }
    
    for (int i = 0; i < content.size(); i++) {
        for (int j = 0; j < content[i].size(); j++) {
            cout << content[i][j] << " ";
        }
        cout << endl;
    }
    
    return 0;
}