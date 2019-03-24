#include<stdio.h>
//#include<stdlib.h>
//#include<array>
//#include<algorithm>
//#include<iostream>

//�}�b�v�f�[�^������W�̃T�C�Y�A
//�����ʒu�A�ڕW�ʒu��subscribe

//���Ƃœ��͒l��������悤�ɕς���
const int w_x = 5;//world�c
const int w_y = 6;//world��
const int init_x = 0;//�����ʒux
const int init_y = 0;//�����ʒuy
const int goal_x = 4;//�ڕW�ʒux
const int goal_y = 5;//�ڕW�ʒuy
const int open_num = 100;//open���X�g�̑傫���v����!!
const int cost = 1;

//�P�����z����Q�����z��ɕϊ�����֐�
void conv_array(int array1[w_x * w_y],int array2[w_x][w_y]){
	for(int i=0;i<w_x;i++){
		for(int j=0;j<w_y:j++){
			array2[i][j] = array1[i*w_y][j];
		}
	}
}




//�z�������������֐�
void init_array(int (&array)[w_x][w_y],int num){
	for(int i=0;i<w_x;i++){
		for(int j=0;j<w_y;j++){
			array[i][j] = num;
		}
	}
}

//�z��ɐ��l���ꊇ�������֐�
void sub_array(int array[],int num1,int num2,int num3,int num4,int num5){
	array[0] = num1;
	array[1] = num2;
	array[2] = num3;
	array[3] = num4;
	array[4] = num5;
}


//100�͋�!�Ƃ݂Ȃ��Ĕz��̒����𔻒肷��֐�
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
//�Q�����z����ŏ��̐��ŏ��������Ƀ\�[�g����֐�
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

//��Βl��Ԃ��֐�
void abs_int(int n){
	if(n<0)
		n *= -1;
	return n;
}


//heuristic�z���������������֐�
void init_heuristic(int array[w_x][w_y]){
	for(int i=0;i<w_x;i++){
		for(int j=0;j<w_y;j++){
			array[i][j] = abs_int(goal_x - i) + abs_int(goal_y - j);
		}
	}
}



//�Q�����z������ɃX���C�h����֐�
void pop_array(int array[open_num][5]){

	for(int i=0;i<open_num-1;i++){
//		std::swap(array[i],array[i+1]);
		for(int j=0;j<5;j++){
			array[i][j] = array[i+1][j];
		}
	}
	array[open_num-1][0] = 100;//2�����z��̍Ō�̔z����󔒂ɂ���
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

	char delta_name[] = {'^','<','v','>'};//�N�I�[�^�j�I���ɂ���
	//�Ƃ肠�����㍶���E�̓�����
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
	//open���X�g��100�ŏ�����
	//100�������Ă���open���X�g�͋�̃��X�g�Ƃ��Ĉ���
	
	sub_array(open[0],f,g,h,x,y);//open[0]��5�̒l�������Ă���
	
	int found = false;//search�������������ǂ���
	int resign = false;//open���X�g����ɂȂ��Ă��܂������ǂ���

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
			pop_array(open);//python�ł���pop�֐�
			
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

	//���ʂ��v�����g�{���͂�����publish������
	for(int i=0;i<w_x;i++){
		printf("\n");
		for(int j=0;j<w_y;j++){
			printf("%3c ",policy[i][j]);
		}
	}

	return 0;

}
