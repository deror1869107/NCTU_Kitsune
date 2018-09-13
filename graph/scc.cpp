#include<bits/stdc++.h>
using namespace std;
#define MAX 10010
vector<int>edge[MAX],group[MAX];
bool instk[MAX];
int stk[MAX],groupID[MAX],nGroup,dfn[MAX],low[MAX],top,nowDfn;
void tarjan(int start){
    dfn[start]=low[start]=++nowDfn;
    instk[start]=1;
    stk[top++]=start;
    for(int i=0;i<edge[start].size();i++){
        int next=edge[start][i];
        if(!dfn[next]){
            tarjan(next);
            if(low[start]>low[next])
                low[start]=low[next];
        }
        if(instk[next])
            if(low[start]>dfn[next])
                low[start]=dfn[next];
    }
    if(dfn[start]==low[start]){
        do{
            --top;
            instk[stk[top]]=0;
            groupID[stk[top]]=nGroup;
            group[nGroup].push_back(stk[top]);
        }while(stk[top]!=start);
        ++nGroup;
    }
}
void init(int n){
    for(int i=0;i<n;i++)
        instk[i]=dfn[i]=0,edge[i].clear(),group[i].clear();
    nowDfn=nGroup=top=0;
}
int main(){
    int T,n,m,i,j,k,x,y;
    while(scanf("%d%d",&n,&m),n||m){
        init(n);
        for(i=0;i<m;i++){
            scanf("%d%d",&x,&y);
            edge[x-1].push_back(y-1);
        }
        for(i=0;i<n;i++)
            if(dfn[i]==0)tarjan(i);
    if(nGroup==1) puts("Yes");
    else puts("No");
    }
  return 0;
}
