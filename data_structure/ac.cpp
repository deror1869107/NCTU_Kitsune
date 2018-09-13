class ACautomaton{
    public:
        class Node{
            public:
                int cnt, depth, no;
                Node *child[26], *fail;
                Node(int _depth = 0){
                    cnt = 0;
                    depth = _depth;
                    no = -1;
                    fail = NULL;
                    memset(child, 0, sizeof(child));
                }
        };

        Node *root, *pool;
        int pp;

        ACautomaton(){
            pool = new Node[1048576];
            pp = 0;
            root = &pool[pp++];
        }

        ~ACautomaton(){
            delete pool;
        }

        void insert(const string &s, int no, Node *nd = NULL, int pos = 0){
            if(nd == NULL)
                nd = root;
            if(pos >= (int) s.size()){
                nd->cnt++;
                nd->no = no;
                return;
            }
            int c = s[pos] - 'a';
            if(nd->child[c] == NULL){
                pool[pp] = Node(nd->depth + 1);
                nd->child[c] = &pool[pp++];
            }
            insert(s, no, nd->child[c], pos+1);
        }

        void build_fail(){
            queue<Node*> q;
            q.push(root);
            while(!q.empty()){
                Node *nd = q.front();
                q.pop();
                for(int i = 0; i < 26; i++){
                    if(nd->child[i]){
                        Node *pt = nd->fail;
                        while(pt && !pt->child[i])
                            pt = pt->fail;
                        nd->child[i]->fail = pt ? pt->child[i] : root;
                        nd->child[i]->cnt += nd->child[i]->fail->cnt;
                        q.push(nd->child[i]);
                    }
                }
            }
        }

        void match(const string &s, int occur[]){
            Node *nd = root;
            for(int i = 0; i < (int) s.size(); i++){
                int c = s[i] - 'a';
                while(nd && !nd->child[c])
                    nd = nd->fail;
                if(nd){
                    nd = nd->child[c];
                    Node *p = nd;
                    while(p && p->cnt){
                        if(p->no >=0 )
                            occur[p->no]++;
                        p = p->fail;
                    }
                }
                else
                    nd = root;
            }
        }
};

int main(){
    string a, b[160];
    int n;
    while(cin >> n, n){
        ACautomaton ac;
        for(int i = 0; i < n; i++){
            cin >> b[i];
            ac.insert(b[i], i);
        }
        cin >> a;
        ac.build_fail();
        int occur[160] = {};
        ac.match(a, occur);
        vector<int> ans;
        int mx = 0;
        for(int i = 0; i < n; i++){
            if(occur[i] > mx){
                ans.clear();
                mx = occur[i];
            }
            if(occur[i] == mx)
                ans.push_back(i);
        }
        cout << mx << endl;
        for(auto i : ans){
            cout << b[i] << endl;
        }
    }
}
