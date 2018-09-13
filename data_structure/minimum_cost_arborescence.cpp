#include <iostream>
#include <cstring>

using namespace std;

const int V = 105, E = 1005;

struct Edge {
    int a, b, c;
    Edge(int a = 0, int b = 0, int c = 0):a(a), b(b), c(c){}
} edge[E];

int d[V];
int p[V];
int v[V];
int n[V];
int m[V];

int main()
{
    cin.tie(0);
    cin.sync_with_stdio(0);
    int vv, e, r;
    cin >> vv >> e >> r;
    int s, t, w;
    for(int i = 0; i < e; ++i){
        cin >> s >> t >> w;
        edge[i] = Edge(s, t, w);
    }
    int w1 = 0, w2 = 0;
    for(int k = 0; k < vv - 1; k++){
        memset(d, 1, sizeof(d));
        memset(p, -1, sizeof(p));
        for(int i = 0; i < e; ++i){
            int& a = edge[i].a;
            int& b = edge[i].b;
            int& c = edge[i].c;
            if(a != b && b != r && c < d[b])
                d[b] = c, p[b] =a;
        }
        memset(v, -1, sizeof(v));
        memset(n, -1, sizeof(n));

        w1 = 0;
        bool jf = false;
        for(int i = 0; i < vv; ++i){
            if(m[i]) continue;
            if(p[i] >= 0) w1 += d[i];

            int s;
            for(s = i; s != -1 && v[s] == -1; s = p[s])
                v[s] = i;
            if(s != -1 && v[s] == i){
                jf = true;
                int j = s;
                do
                {
                    n[j] = s; m[j] = 1;
                    w2 += d[j]; j = p[j];
                } while(j != s);
                m[s] = 0;
            }
        }
        if(!jf) break;
        for(int i = 0; i < e; ++i){
            int& a = edge[i].a;
            int& b = edge[i].b;
            int& c = edge[i].c;
            if(n[b] >= 0) c -= d[b];
            if(n[a] >= 0) a = n[a];
            if(n[b] >= 0) b = n[b];
            if(a == b) edge[i--] = edge[--e];
        }
    }
    cout << w1 + w2 << endl;
}
