int d[MAX + 1];

void init() {
    for(int i = 1; i <= MAX; ++i) {
        d[i] = i;
    }
}

int find(int n) {
    return (d[n] == n) ? d[n] : d[n] = find(d[n]);
}

bool union(int a, int b) {
    int x = find(a);
    int y = find(b);
    if(x != y) d[x] = y;
    return x != y;  // for Kruskal
}
