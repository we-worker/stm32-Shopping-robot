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
int16_t pre[N * N]; //倒序路径结果
int t_dir[N * N]; //倒序方向
int dis[N][N];;

int show_dir[N * N] = {0};
int show_index = 0;

void swap(int *a, int *b)
{
	int t = *a;
	*a = *b;
	*b = t;
}

void dfs(int start_x, int start_y, int end_x, int end_y)
{
	if ((start_x == end_x && start_y == end_y) || vis[start_x][start_y])
		return;
	vis[start_x][start_y] = 1;
	for (int i = 0; i < 4; i++)
	{
		int new_x = start_x + X[i];
		int new_y = start_y + Y[i];

		if (new_x >= 0 && new_x < N && new_y >= 0 && new_y < N && !Gi[new_x][new_y]) //地图边界和障碍物判断
		{
			swap(&X[0], &X[i]);
			swap(&Y[0], &Y[i]);
			if (dis[new_x][new_y] > dis[start_x][start_y] + 1)
			{

				pre[new_x * N + new_y] = start_x * N + start_y;
				
				
				if (X[i] == 0 && Y[i] == 1)
					t_dir[new_x * N + new_y] = 2;
				else if (X[i] == 0 && Y[i] == -1)
					t_dir[new_x * N + new_y] = 4;
				else if (X[i] == 1 && Y[i] == 0)
					t_dir[new_x * N + new_y] = 1;
				else
					t_dir[new_x * N + new_y] = 3;
				
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

		//printf("%d %d\n", n / N, n % N);
		printf("dir:%d\n", t_dir[n]);
		show_dir[show_index++] = t_dir[n];
		return;
	}
	re_get(pre[n], ed);

	//printf("%d %d\n", n / N, n % N);
	printf("dir:%d\n", t_dir[n]);
	show_dir[show_index++] = t_dir[n];
	

}

void print(){
	for(int i=0;i<show_index;i++)
		printf("show_dir[%d]:%d\n",i, t_dir[i]);
}

void solve(int sx, int sy, int ex, int ey)
{
	for (int i = 0; i < N; i++)
		for (int j = 0; j < N; j++)
			dis[i][j] = 60000;
	dis[sx][sy] = 0;
	dfs(sx, sy, ex, ey);
	re_get(ex * N + ey, sx * N + sy);
	print();
}

