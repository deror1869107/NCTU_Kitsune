struct node {
    int data, s, tag, sum, mn;
    bool rev;
    node *l, *r;
    node(const int d = 0):data(d), s(1), l(0), r(0), sum(d), tag(0), mn(d), rev(false){}

    inline void up()
    {
        s = 1;
        sum = mn = data;
        if(l){
            s += l->s;
            sum += (l->sum + l->tag);
            mn = min(mn, l->mn + l->tag);
        }
        if(r){
            s += r->s;
            sum += (r->sum + r->tag);
            mn = min(mn, r->mn + r->tag);
        }
    }

    inline void down()
    {
        if(rev){
            rev = false;
            if(l) l->rev ^= 1;
            if(r) r->rev ^= 1;
            swap(l, r);
        }
        data += tag;
        if(l) l->tag += tag;
        if(r) r->tag += tag;
        tag = 0;
    }
};

int NN = 0;
node pool[50005];

inline int size(node *o){ return o?o->s:0; }

void split(node *o, node *&a, node *&b, int k)
{
    if(!o) a = b = 0;
    else {
        o->down();
        if(k <= size(o->l)){
            b = o;
            split(o->l, a, b->l, k);
        } else {
            a = o;
            split(o->r, a->r, b, k - size(o->l) - 1);
        }
        o->up();
    }
}

node *merge(node *a, node *b)
{
    if(!a || !b) return a?a:b;
    static int x;
    if(x++%(a->s+b->s)<a->s){
        a->down();
        a->r=merge(a->r,b);
        a->up();
        return a;
    } else {
        b->down();
        b->l=merge(a,b->l);
        b->up();
        return b;
    }
}

void build(node *&root, int n)
{
    int a;
    node *now;
    for(int i = 0; i < n; ++i){
        scanf("%d", &a);
        pool[NN] = node(a);
        now = &pool[NN++];
        root = merge(root, now);
    }
}

void init(){ NN = 0; }

int main()
{
    int T;
    scanf("%d", &T);
    for(int tc = 1; tc <= T; ++tc){
        printf("Case %d:\n", tc);
        int N;
        scanf("%d", &N);
        node *root = NULL;
        init();
        build(root, N);
        char cmd[6];
        int i, j;
        node *n1, *n2, *n3;
        while(true){
            scanf("%s", cmd);
            if(cmd[0] == 'A'){ // ADD
                scanf("%d %d", &i, &j);
                split(root, n1, n2, i - 1);
                split(n2, n2, n3, 1);
                n2->tag += j;
                root = merge(merge(n1, n2), n3);
            } else if(cmd[0] == 'S'){ // SUB
                scanf("%d %d", &i, &j);
                split(root, n1, n2, i - 1);
                split(n2, n2, n3, 1);
                n2->tag -= j;
                root = merge(merge(n1, n2), n3);
            } else if(cmd[0] == 'Q'){ // QUERY
                scanf("%d %d", &i, &j);
                split(root, n1, n2, i - 1);
                split(n2, n2, n3, j - i + 1);
                printf("%d\n", n2->sum);
                root = merge(merge(n1, n2), n3);
            } else {
                break;
            }
        }
    }
}
