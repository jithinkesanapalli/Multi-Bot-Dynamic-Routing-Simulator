#include <iostream>
#include <vector>
#include <queue>
#include <cstdlib>
#include <algorithm>
#include <cmath>
#include <map>
#include <sstream>
#include <fstream>
#include <limits>
#include <string>
#include <functional>
#include <ctime>

using namespace std;

// colours for show output in terminal
#define RESET   "\033[0m"
#define RED     "\033[31m"
#define GREEN   "\033[32m"
#define YELLOW  "\033[33m"
#define BLUE    "\033[34m"
#define MAGENTA "\033[35m"
#define CYAN    "\033[36m"
#define WHITE   "\033[37m"
#define GRAY    "\033[90m"

// grid size vars
int ROWS;
int COLS;

// grid data 0=free 1=block 2=start 3=goal
vector<vector<int>> grid;

// delay use for show animation
#ifdef _WIN32
#include <windows.h>
void delay(int ms) { Sleep(ms); }
#else
#include <unistd.h>
void delay(int ms) { usleep(ms * 1000); }
#endif

// bot states
enum BotState {
    MOVING,
    WAITING,
    REACHED,
    RE_PLANNING
};

// bot info store here
struct Bot {
    int id;
    int r, c;
    int goalR, goalC;
    vector<pair<int,int>> path;
    size_t step;
    char symbol;
    string color;
    BotState state;
    int wait_counter;
};

// forward declare
vector<vector<int>> makeGraph(const vector<pair<int, int>>& temp_obstacles = {});
vector<pair<int,int>> dijkstra(int sr, int sc, int gr, int gc, const vector<pair<int, int>>& temp_obstacles = {});
void handle_replanning(Bot& bot, int time_step);

// make grid with random walls
void makeGrid() {
    grid.assign(ROWS, vector<int>(COLS, 0));

    srand(42); // fixed so same always
    int obsCount = (ROWS * COLS) / 5;
    for (int k = 0; k < obsCount; k++) {
        int r = rand() % ROWS;
        int c = rand() % COLS;
        if (grid[r][c] == 0) grid[r][c] = 1; // put block
    }
}

// create graph edges from grid
vector<vector<int>> makeGraph(const vector<pair<int, int>>& temp_obstacles) {
    vector<vector<int>> adj(ROWS * COLS);
    int dr[4] = {-1, 1, 0, 0};
    int dc[4] = {0, 0, -1, 1};

    // check if cell blocked
    auto is_blocked = [&](int r, int c) {
        if (grid[r][c] == 1) return true;
        for (const auto& obs : temp_obstacles) {
            if (obs.first == r && obs.second == c) return true;
        }
        return false;
    };

    // build edges for neighbor cells
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            if (is_blocked(r, c)) continue;
            int id = r * COLS + c;

            for (int i = 0; i < 4; i++) {
                int nr = r + dr[i];
                int nc = c + dc[i];

                if (nr >= 0 && nr < ROWS && nc >= 0 && nc < COLS && !is_blocked(nr, nc)) {
                    int nid = nr * COLS + nc;
                    adj[id].push_back(nid);
                }
            }
        }
    }
    return adj;
}

// dijkstra algo find shortest steps
vector<pair<int,int>> dijkstra(int sr, int sc, int gr, int gc, const vector<pair<int, int>>& temp_obstacles) {
    if (sr < 0 || sr >= ROWS || sc < 0 || sc >= COLS) return {};
    if (gr < 0 || gr >= ROWS || gc < 0 || gc >= COLS) return {};
    if (grid[sr][sc] == 1) return {};
    if (grid[gr][gc] == 1) return {};

    vector<vector<int>> adj = makeGraph(temp_obstacles);
    int start = sr * COLS + sc;
    int goal = gr * COLS + gc;
    int total = ROWS * COLS;

    const int INF = 1e9;
    vector<int> dist(total, INF);
    vector<int> parent(total, -1);
    dist[start] = 0;

    priority_queue<pair<int,int>, vector<pair<int,int>>, greater<pair<int,int>>> pq;
    pq.push({0, start});

    while (!pq.empty()) {
        int d = pq.top().first;
        int node = pq.top().second;
        pq.pop();

        if (d > dist[node]) continue;
        if (node == goal) break;

        for (int nei : adj[node]) {
            int nd = dist[node] + 1;
            if (nd < dist[nei]) {
                dist[nei] = nd;
                parent[nei] = node;
                pq.push({nd, nei});
            }
        }
    }

    if (dist[goal] == INF) return {};

    vector<pair<int,int>> path;
    int cur = goal;
    while (cur != -1) {
        path.push_back({cur / COLS, cur % COLS});
        cur = parent[cur];
    }
    reverse(path.begin(), path.end());
    return path;
}

// clear screen and print grid + bots
void printGrid(vector<Bot>& bots, int time_step) {
#ifdef _WIN32
    system("cls");
#else
    system("clear");
#endif

    vector<vector<string>> show(ROWS, vector<string>(COLS, "."));

    // show blocks start goal
    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            if (grid[r][c] == 1) show[r][c] = string(GRAY) + "#" + RESET;
            else if (grid[r][c] == 2) show[r][c] = string(WHITE) + "W" + RESET;
            else if (grid[r][c] == 3) show[r][c] = string(GREEN) + "G" + RESET;
            else show[r][c] = ".";
        }
    }

    // show each bot
    for (auto& b : bots) {
        if (b.state != REACHED)
            show[b.r][b.c] = b.color + b.symbol + RESET;
    }

    cout << "=== warehouse sim " << ROWS << "x" << COLS << " ===\n";
    cout << "step: " << time_step << "\n\n";

    for (int r = 0; r < ROWS; r++) {
        for (int c = 0; c < COLS; c++) {
            cout << show[r][c] << " ";
        }
        cout << "\n";
    }
    cout << "\n";
}

// when bot got stuck then recalc path
void handle_replanning(Bot& bot, int time_step) {
    int sr = bot.r, sc = bot.c;
    int gr = bot.goalR, gc = bot.goalC;

    vector<pair<int, int>> temp_obstacles; // empty for now

    bot.path = dijkstra(sr, sc, gr, gc, temp_obstacles);

    if (!bot.path.empty()) {
        bot.step = 0;
        bot.state = MOVING;
    } else {
        bot.state = WAITING;
        bot.wait_counter = 10; // wait some time
    }
}

// ask user for valid cell posn
void getValidPosition(int& outR, int& outC, const string& prompt) {
    while (true) {
        int r, c;
        cout << prompt << " enter row col: ";
        if (!(cin >> r >> c)) {
            cin.clear();
            cin.ignore(numeric_limits<streamsize>::max(), '\n');
            cout << RED << "bad input try again" << RESET << endl;
            continue;
        }
        if (r < 0 || r >= ROWS || c < 0 || c >= COLS) {
            cout << RED << "outside grid try again" << RESET << endl;
            continue;
        }
        if (grid[r][c] == 1) {
            cout << RED << "this is blocked try again" << RESET << endl;
            continue;
        }
        outR = r;
        outC = c;
        return;
    }
}

int main() {
    cout << "--- bot routing setup ---\n";
    cout << "enter rows: ";
    if (!(cin >> ROWS)) ROWS = 30;
    cout << "enter cols: ";
    if (!(cin >> COLS)) COLS = 30;

    if (ROWS <= 0 || COLS <= 0) {
        cerr << RED << "bad grid size" << RESET << endl;
        return 1;
    }

    int num_bots;
    cout << "enter bot count (1-5): ";
    if (!(cin >> num_bots) || num_bots < 1 || num_bots > 5) {
        num_bots = 3;
        cout << YELLOW << "bad input using 3" << RESET << endl;
    }

    makeGrid();

    vector<Bot> bots;
    vector<char> symbols = {'A','B','C','D','E'};
    vector<string> colors = {RED,BLUE,YELLOW,MAGENTA,CYAN};

    cout << "\n--- set start and goal ---\n";
    for (int i = 0; i < num_bots; ++i) {
        int r, c, goalR, goalC;

        cout << "\nbot " << i+1 << " (" << symbols[i] << ")\n";

        getValidPosition(r, c, "start");
        grid[r][c] = 2; // mark start

        while (true) {
            getValidPosition(goalR, goalC, "goal");
            if (goalR == r && goalC == c) {
                cout << RED << "goal cannot same as start" << RESET << endl;
                continue;
            }
            break;
        }

        grid[goalR][goalC] = 3; // mark goal

        Bot b;
        b.id = i+1;
        b.r = r; b.c = c;
        b.goalR = goalR; b.goalC = goalC;
        b.path = {};
        b.step = 0;
        b.symbol = symbols[i];
        b.color = colors[i];
        b.state = MOVING;
        b.wait_counter = 0;

        bots.push_back(b);
    }

    cout << "\n--- calc first paths ---\n";
    for (auto& b : bots) {
        b.path = dijkstra(b.r, b.c, b.goalR, b.goalC, {});
        if (b.path.empty()) {
            cout << RED << "bot " << b.id << " no path exiting" << RESET << endl;
            return 0;
        }
        b.step = 0;
    }

    bool allDone = false;
    int time_step = 0;

    // main loop
    while (!allDone) {
        allDone = true;
        time_step++;

        // handle replanning and wait logic
        for (auto& bot : bots) {
            if (bot.state == RE_PLANNING)
                handle_replanning(bot, time_step);

            if (bot.state == WAITING) {
                if (bot.wait_counter > 0) bot.wait_counter--;
                else bot.state = MOVING;
            }
        }

        // record next moves
        map<int, pair<int,int>> nextPosIntent;
        for (auto& b : bots) {
            pair<int,int> next;
            if (b.state == REACHED || b.state == WAITING) {
                next = {b.r, b.c};
            } else if (b.step + 1 < b.path.size()) {
                next = b.path[b.step+1];
            } else {
                next = {b.r, b.c};
            }
            nextPosIntent[b.id] = next;
        }

        map<pair<int,int>, int> executedPosReservation;

        // move bots one by one
        for (auto& bot : bots) {
            if (bot.state != MOVING) {
                if (bot.state != REACHED) allDone = false;
                continue;
            }

            pair<int,int> nextPos = nextPosIntent[bot.id];
            pair<int,int> currentPos = {bot.r, bot.c};
            bool canMove = true;

            // check if someone already move there
            if (executedPosReservation.count(nextPos))
                canMove = false;

            // check swap conflict
            if (canMove && executedPosReservation.count(currentPos)) {
                int other_id = executedPosReservation[currentPos];
                auto it = find_if(bots.begin(), bots.end(),
                                  [&](const Bot& b){ return b.id == other_id; });

                if (it != bots.end() && it->step > 0 && it->step < it->path.size()) {
                    pair<int,int> prev = it->path[it->step - 1];
                    if (nextPos == prev)
                        canMove = false;
                }
            }

            if (!canMove) {
                bot.state = WAITING;
                bot.wait_counter = 1;
            } else {
                if (bot.step + 1 < bot.path.size()) {
                    bot.step++;
                    bot.r = bot.path[bot.step].first;
                    bot.c = bot.path[bot.step].second;
                }
                executedPosReservation[{bot.r, bot.c}] = bot.id;
            }

            if (bot.r == bot.goalR && bot.c == bot.goalC)
                bot.state = REACHED;

            if (bot.state != REACHED) allDone = false;
        }

        printGrid(bots, time_step);

        cout << "bot status:\n";
        for (auto& b : bots) {
            cout << b.color << "bot " << b.symbol << RESET
                 << " (" << b.r << "," << b.c << ")";
            if (b.state == REACHED) cout << " reached";
            else if (b.state == WAITING) cout << " waiting " << b.wait_counter;
            else if (b.state == RE_PLANNING) cout << " replanning";
            else cout << " moving";
            cout << "\n";
        }

        delay(250);
    }

    cout << GREEN << "\nall bot reach goal in " << time_step << " steps\n" << RESET;
    return 0;
}
