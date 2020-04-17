#include <iostream>
#include <sstream>
#include <fstream>
#include <vector>
#include <algorithm>

using namespace std;

// enum of State
enum class State {kEmpty, kObstacle,kClosed,kPath,kStart,kFinish};

// left,up,right,down
const int delta[4][2]={{-1,0},{0,-1},{1,0},{0,1}};

//Parse line to State
vector<State> ParseLine(string line){
    istringstream s_stream(line);
    int a;
    char ch;
    vector<State> lineState;
    while(s_stream>>a>>ch && ch==','){
        if(a==1)lineState.push_back(State::kObstacle);
        else lineState.push_back(State::kEmpty);
    }
    return lineState;
}

//read file and return 2d grid
vector<vector<State>> ReadBoardFile(string path){
    ifstream infile(path);
    string line;
    vector<vector<State>> board;
    while (infile>>line)
    {
        vector<State> lineState = ParseLine(line);
        board.push_back(lineState);
    }
    return board;
}

//compute Heuristic with Manhattan Distance
int Heuristic(int x1,int y1,int x2,int y2){
    return abs(x1-x2)+abs(y1-y2);
}

//add f to openList
void AddToOpen(int x,int y,int g,int h,vector<vector<int>> &openList,vector<vector<State>> &board){
    openList.push_back(vector<int>{x,y,g,h});
    board[x][y] = State::kClosed;
}

//Compare 
bool Compare(vector<int> node1,vector<int> node2){
    return node1[2]+node1[3]>node2[2]+node2[3]?1:0;
}

// cellSort
void CellSort(vector<vector<int>> *v){
    sort(v->begin(),v->end(),Compare);
}

bool CheckValidCell(int x,int y,vector<vector<State>> board){
    if(x>=0 && x<board.size() && y>=0 && y<board[0].size() && board[x][y]==State::kEmpty)return true;
    else return false;
}

// check 4-connected neighbors and add to openList
void ExpandNeighbors(const vector<int> &current,int goal[2],vector<vector<int>> &openList,vector<vector<State>> &board){
    int x = current[0];
    int y = current[1];
    int g = current[2];
    for(int i=0;i<4;i++){
        int x2 = x + delta[i][0];
        int y2 = y + delta[i][1];
        if(CheckValidCell(x2,y2,board)){
            int g2 = g+1;
            int h2 = Heuristic(x2,y2,goal[0],goal[1]);
            AddToOpen(x2,y2,g2,h2,openList,board);
        }
    }
}

//A* search 
vector<vector<State>> Search(vector<vector<State>> &board,int init[2],int goal[2]){
    int x1 = init[0],y1 = init[1];
    int x2 = goal[0],y2 = goal[1];
    int h = Heuristic(x1,y1,x2,y2);
    int g = 0;
    vector<vector<int>> openList;
    AddToOpen(x1,y1,g,h,openList,board);

    while(openList.size()>0){
        // find the one with minimal f value
        CellSort(&openList);
        auto current = openList.back();
        openList.pop_back();
        x1 = current[0];
        y1 = current[1];
        board[x1][y1] = State::kPath;
        
        if(x1==goal[0] && y1==goal[1]){
            board[init[0]][init[1]] = State::kStart;
            board[goal[0]][goal[1]] = State::kFinish;
            return board;
        }

        ExpandNeighbors(current,goal,openList,board);
    }
    return vector<vector<State>>{};
}

string CellString(State cell) {
  switch(cell) {
    case State::kObstacle: return "⛰️   ";
    case State::kPath: return "🚗   ";
    case State::kStart: return "🚦   ";
    case State::kFinish: return "🏁   ";
    default: return "0    "; 
  }
}

void PrintBoard(vector<vector<State>> board){
    for(auto i:board){
        for(auto j:i){
            cout << CellString(j);
        }
        cout << "\n";
    }
}



int main(){
    vector<vector<State>> board = ReadBoardFile("1.board");
    int start[2]={0,0},goal[2]={4,5};

    vector<vector<State>> res = Search(board,start,goal);

    PrintBoard(res);
    
    return 0;
}