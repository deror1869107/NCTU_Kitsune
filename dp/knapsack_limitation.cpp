int v[100 + 1], w[100 + 1], m[100 + 1];
int dp[10000 + 1];

int knapsack(int N, int W)
{
    int ans = 0;
    for(int i = 0; i < N; ++i) cin >> v[i] >> w[i] >> m[i];
    for(int i = 0; i < N; ++i)
    {
        for(int j = 0; m[i] > 0; ++j)
        {
            int take = min(m[i], (1 << j));
            m[i] -= take;
            for(int k = W; k >= take * w[i]; --k) dp[k] = max(dp[k], dp[k - take * w[i]] + take * v[i]);
        }
    }
    for(int i = W; i >= 0; --i) ans = max(ans, dp[i]);
    return ans;
}
