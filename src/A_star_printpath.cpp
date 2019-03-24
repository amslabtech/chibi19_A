#include<stdio.h>
//#include<stdlib.h>
//#include<array>
//#include<algorithm>
//#include<iostream>

//マップデータから座標のサイズ、
//初期位置、目標位置をsubscribe

//あとで入力値を代入するように変える
const int w_x = 5;//world縦
const int w_y = 6;//world横
const int init_x = 0;//初期位置x
const int init_y = 0;//初期位置y
const int goal_x = 4;//目標位置x
const int goal_y = 5;//目標位置y
const int open_num = 100;//openリストの大きさ要検討!!
const int cost = 1;

//１次元配列を２次元配列に変換する関数
void conv_array(int array1[w_x * w_y],int array2[w_x][w_y]){
	for(int i=0;i<w_x;i++){
		for(int j=0;j<w_y:j++){
			array2[i][j] = array1[i*w_y][j];
		}
	}
}




//配列を初期化する関数
void init_array(int (&array)[w_x][w_y],int num){
	for(int i=0;i<w_x;i++){
		for(int j=0;j<w_y;j++){
			array[i][j] = num;
		}
	}
}

//配列に数値を一括代入する関数
void sub_array(int array[],int num1,int num2,int num3,int num4,int num5){
	array[0] = num1;
	array[1] = num2;
	array[2] = num3;
	array[3] = num4;
	array[4] = num5;
}


//100は空白!とみなして配列の長さを判定する関数
int len_array(int array[open_num][5]){

	int count = 0;
	for(int i=0;i<open_num;i++){
		if(array[i][0] == 100){
			break;
		}
		count++;
	}
	return count;
}
//２次元配列を最初の数で小さい順にソートする関数
void sort_array(int array[open_num][5]){
//	std::sort(&array[0],&array[len_array(array)],[](auto& x,auto& y){return x[0] < y[0];});
	int temp[5];
	
	for(int i=0;i<open_num;i++){
		for(int j=0;j<open_num-1;j++){
			if(array[j][0] > array[j+1][0]){
				for(int k=0;k<5;k++){
					temp[k] = array[j][k];
					array[j][k] = array[j+1][k];
					array[j+1][k] = temp[k];
				}
			}
		}
	}
}

//絶対値を返す関数
void abs_int(int n){
	if(n<0)
		n *= -1;
	return n;
}


//heuristic配列を自動生成する関数
void init_heuristic(int array[w_x][w_y]){
	for(int i=0;i<w_x;i++){
		for(int j=0;j<w_y;j++){
			array[i][j] = abs_int(goal_x - i) + abs_int(goal_y - j);
		}
	}
}



//２次元配列を左にスライドする関数
void pop_array(int array[open_num][5]){

	for(int i=0;i<open_num-1;i++){
//		std::swap(array[i],array[i+1]);
		for(int j=0;j<5;j++){
			array[i][j] = array[i+1][j];
		}
	}
	array[open_num-1][0] = 100;//2次元配列の最後の配列を空白にする
}


int main(void){

	int grid[w_x][w_y] = {	{0,1,0,0,0,0},
							{0,1,0,0,0,0},
							{0,1,0,0,0,0},
							{0,1,0,0,0,0},
							{0,0,0,0,1,0}	};

	int heuristic[w_x][w_y] = {	{9,8,7,6,5,4},
								{8,7,6,5,4,3},
								{7,6,5,4,3,2},
								{6,5,4,3,2,1},
								{5,4,3,2,1,0}	};
	int delta[4][2] = {	{-1,0},
						{0,-1},
						{1,0},
						{0,1}	};
	int closed[w_x][w_y];
	init_array(closed,0);
	closed[init_x][init_y] = 1;

	char delta_name[] = {'^','<','v','>'};//クオータニオンにする
	//とりあえず上左下右の頭文字
	int expand[w_x][w_y];
	int action[w_x][w_y];
	char policy[w_x][w_y];

	init_array(expand,-1);
	init_array(action,-1);

	for(int i=0;i<w_x;i++){
		for(int j=0;j<w_y;j++){
			policy[i][j] = ' ';
		}
	}

	int x = init_x;
	int y = init_y;
	int g = 0;
	int h = heuristic[x][y];
	int f = g + h;
	int count = 0;
	int next[5];
	int x2;
	int y2;
	int g2;
	int h2;
	int f2;

	//std::array<std::array<int,5>,open_num> open;
	int open[open_num][5];

	for(int i=0;i<open_num;i++){
		for(int j=0;j<5;j++){
			open[i][j] = 100;
		}
	}
	//openリストを100で初期化
	//100が入っているopenリストは空のリストとして扱う
	
	sub_array(open[0],f,g,h,x,y);//open[0]に5つの値を代入している
	
	int found = false;//searchが完了したかどうか
	int resign = false;//openリストが空になってしまったかどうか

	while(found==false && resign==false){
		if(len_array(open) == 0){
			resign = true;
			printf("\nfail\n");
			return 0;
		}

		else{
			sort_array(open);
			for(int i=0;i<5;i++){
				next[i] = open[0][i];
			}
			pop_array(open);//pythonでいうpop関数
			
			x = next[3];
			y = next[4];
			g = next[1];
			expand[x][y] = count;
			count++;

			if(x==goal_x && y==goal_y){
				found = true;
			}

			else{
				for(int i=0;i<4;i++){
					x2 = x + delta[i][0];
					y2 = y + delta[i][1];
					if(x2>=0 && x2<w_x && y2>=0 && y2<w_y){
						if(closed[x2][y2]==0 && grid[x2][y2]==0){
							g2 = g + cost;
							h2 = heuristic[x2][y2];
							f2 = g2 + h2;
							sub_array(open[len_array(open)],f2,g2,h2,x2,y2);
							closed[x2][y2] = 1;
							action[x2][y2] = i;
						}
					}
				}
			}
		}
	}

	x = goal_x;
	y = goal_y;
	policy[x][y] = '*';

	while(x!=init_x || y!=init_y){
		x2 = x - delta[action[x][y]][0];
		y2 = y - delta[action[x][y]][1];
		policy[x2][y2] = delta_name[action[x][y]];
		x = x2;
		y = y2;
	}

	//結果をプリント本当はここでpublishしたい
	for(int i=0;i<w_x;i++){
		printf("\n");
		for(int j=0;j<w_y;j++){
			printf("%3c ",policy[i][j]);
		}
	}

	return 0;

}
