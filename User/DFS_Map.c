#include "stdio.h"
#include "DFS_Map.h"
#include "stm32f4xx.h"

#define N 10

int Gi[N][N] = {
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
    {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
    {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
    {0, 0, 0, 1, 1, 1, 1, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
};
int vis[N][N];
int X[] = {0, 0, 1, -1};
int Y[] = {1, -1, 0, 0};
int pre[N * N] = {0};   //倒序路径结果
int t_dir[N * N] = {0}; //倒序方向
int dis[N][N];

// int show_dir[N * N] = {0};
// int show_index = 0;
int Route[N * N][2];
int Route_index = 0;

int Turn[N * N]; //转向数组
int Car2_pos_R[4][2] = {{1, 0}, {0, 1}, {-1, 0}, {0, -1}};
int Car2_pos[2] = {1, 0};
int Car_pos[2] = {3, 0};

void swap(int *a, int *b)
{
    int t = *a;
    *a = *b;
    *b = t;
}

void Get_dir(int *show_dir, int *show_index, int sx, int sy, int ex, int ey)
{
    int to_x = ex - sx;
    int to_y = ey - sy;
    if (to_x == 0 && to_y == 1)
        show_dir[(*show_index)++] = 2;
    else if (to_x == 0 && to_y == -1)
        show_dir[(*show_index)++] = 4;
    else if (to_x == 1 && to_y == 0)
        show_dir[(*show_index)++] = 1;
    else
        show_dir[(*show_index)++] = 3;
}

void dfs(int start_x, int start_y, int end_x, int end_y)
{
    if (start_x == end_x && start_y == end_y || vis[start_x][start_y])
        return;
    vis[start_x][start_y] = 1;
    for (int i = 0; i < 4; i++)
    {
        int new_x = start_x + X[i];
        int new_y = start_y + Y[i];

        /*这里用来判断能不能转向
         */
        // if (start_x == 7 && (start_y >= 3 && start_y <= 6) && new_x == start_x + 1)
        // {
        //     continue; //在下侧时，不能再向下。
        // }

        if (new_x >= 0 && new_x < N && new_y >= 0 && new_y < N && !Gi[new_x][new_y]) //地图边界和障碍物判断
        {

            swap(&X[0], &X[i]);
            swap(&Y[0], &Y[i]);
            if (dis[new_x][new_y] > dis[start_x][start_y] + 1)
            {
                pre[new_x * N + new_y] = start_x * N + start_y;
                dis[new_x][new_y] = dis[start_x][start_y] + 1;
                dfs(new_x, new_y, end_x, end_y);
            }
            swap(&X[0], &X[i]);
            swap(&Y[0], &Y[i]);
        }
    }
    vis[start_x][start_y] = 0;
}

void re_get(int n, int ed)
{

    if (n == ed)
    {
        Route[Route_index][0] = n / N;
        Route[Route_index][1] = n % N;
        Route_index++;
        //printf("%d %d\n", n / N, n % N);
        return;
    }
    re_get(pre[n], ed);
    //printf("%d %d\n", n / N, n % N);
    Route[Route_index][0] = n / N;
    Route[Route_index][1] = n % N;
    Route_index++;
}
void solve(int *show_dir, int *show_index, int sx, int sy, int ex, int ey)
{
    for (int i = 0; i < N; i++)
        for (int j = 0; j < N; j++)
            dis[i][j] = 60000;
    dis[sx][sy] = 0;
	Route_index = 0;
    dfs(sx, sy, ex, ey);
    re_get(ex * N + ey, sx * N + sy);
    for (int i = 0; i < Route_index - 1; i++)
    {
        Get_dir(show_dir, show_index, Route[i][0], Route[i][1], Route[i + 1][0], Route[i + 1][1]);
    }

    // printf("hell00");
     //for (int i = 0; i < *show_index; i++)
     //{
     //    printf("dir:%d\n", show_dir[i]);
     //}
    
}


