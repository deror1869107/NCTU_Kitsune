struct Node {int b, d;};
bool operator<(const Node& n1, const Node& n2) {
  return n1.d > n2.d;
}
const int N;
vector<Node> edge[N];    // adjacency list
int d[N];
int parent[N];
bool visit[N];
void dijkstra(int source)
{
    for (int i=0; i<N; i++) visit[i] = false;
    for (int i=0; i<N; i++) d[i] = 1e9;
    priority_queue<Node> PQ;
    d[source] = 0;
    parent[source] = source;
    PQ.push((Node){source, d[source]});
    for (int i=0; i<N; i++)
    {
        int a = -1;
        while (!PQ.empty() && visit[a = PQ.top().b])
            PQ.pop();
        if (a == -1) break;
        visit[a] = true;
    for(auto& e : edge[a]){
            if(!visit[e.b] && d[a] + e.d < d[e.b]){
                d[e.b] = d[a] + e.d;
                PQ.push(Node(e.b, d[e.b]));
            }
        }
    }
}
