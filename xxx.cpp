#include <iostream>
#include <vector>
using namespace std;

int main()
{
    int n;
    cin>>n;
    for (int i=0;i<n;i++)
    {
        cout<<"hellosdsd git"<<endl;
    }

    vector<int> vec(n,0);
    for (auto iter=vec.begin(); iter!=vec.end();iter++)
    {
        *iter=iter-vec.begin();
        cout<< *iter <<endl;
    }
    return 0;
}
