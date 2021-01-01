#include <iostream>
#include <random>
#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine/olcPixelGameEngine.h"

constexpr float PI = 3.14159;
constexpr float FLOAT_MIN = 0.0f;
constexpr float FLOAT_MAX = 1.0f;

std::random_device rd;
std::default_random_engine eng(rd());
std::uniform_real_distribution<float> distr(FLOAT_MIN, FLOAT_MAX);

const int blockSize = 20;

float dist(olc::vi2d &v1, olc::vi2d &v2)
{
    return sqrtf((v1.x - v2.x) * (v1.x - v2.x) + (v1.y - v2.y) * (v1.y - v2.y));
}

class AStarPathFinding : public olc::PixelGameEngine
{
private:
    enum CellState
    {
        open,
        blocked,
        start,
        end
    };

    struct Cell
    {
        int x, y;

        CellState state = open;

        float localScore = INFINITY;
        float globalScore = INFINITY;
        bool visited = false;

        std::vector<Cell *> neighbors;
        Cell *parent = NULL;
    };

    Cell *cells;

    int cols;
    int rows;
    int length;

    int startX, startY, endX, endY;

    std::vector<Cell *> openSet;
    Cell *currentSolution = NULL;

    bool solved = false;

    void reset()
    {
        currentSolution = NULL;
        solved = false;
        openSet.clear();

        // build initial cells
        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                Cell cell;
                cell.x = x;
                cell.y = y;
                cell.state = distr(eng) < 0.2f ? blocked : open;
                cells[y * cols + x] = cell;
            }
        }

        // calculate neighbors
        auto p = [&](int x, int y) { return y * cols + x; };
        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                Cell *cell = &cells[p(x, y)];
                if (y > 0)
                {
                    cell->neighbors.push_back(&cells[p(x, y - 1)]);
                }

                if (y < rows - 1)
                {
                    cell->neighbors.push_back(&cells[p(x, y + 1)]);
                }

                if (x > 0)
                {
                    cell->neighbors.push_back(&cells[p(x - 1, y)]);
                }

                if (x < cols - 1)
                {
                    cell->neighbors.push_back(&cells[p(x + 1, y)]);
                }

                // allow diag moves
                // if (x > 0 && y > 0)
                // {
                //     cell->neighbors.push_back(&cells[p(x - 1, y - 1)]);
                // }

                // if (x < cols - 1 && y > 0)
                // {
                //     cell->neighbors.push_back(&cells[p(x + 1, y - 1)]);
                // }

                // if (x < cols - 1 && y < rows - 1)
                // {
                //     cell->neighbors.push_back(&cells[p(x + 1, y + 1)]);
                // }

                // if (x > 0 && y < rows - 1)
                // {
                //     cell->neighbors.push_back(&cells[p(x - 1, y + 1)]);
                // }
            }
        }

        cells[startY * cols + startX].state = start;
        cells[startY * cols + startX].localScore = 0;

        olc::vi2d v1 = {startX, startY};
        olc::vi2d v2 = {endX, endY};
        cells[startY * cols + startX].globalScore = dist(v1, v2);

        cells[endY * cols + endX].state = end;

        openSet.push_back(&cells[p(startX, startY)]);
    }

    void astarStep()
    {
        std::sort(openSet.begin(), openSet.end(), [](Cell *c1, Cell *c2) {
            return c1->globalScore < c2->globalScore;
        });

        while (!openSet.empty() && openSet.front()->visited)
            openSet.erase(openSet.begin());

        if (openSet.empty())
            return;

        Cell *current = openSet.front();
        current->visited = true;
        openSet.erase(openSet.begin());

        if (current->x == endX && current->y == endY)
        {
            currentSolution = current;
            solved = true;
            return;
        }

        olc::vi2d currentLocation = {current->x, current->y};
        olc::vi2d endLocation = {endX, endY};
        for (auto &neighbor : current->neighbors)
        {
            if (!neighbor->visited && neighbor->state != blocked)
                openSet.push_back(neighbor);

            olc::vi2d neighborLocation = {neighbor->x, neighbor->y};

            float possiblyNewScore = current->localScore + dist(currentLocation, neighborLocation);

            if (possiblyNewScore < neighbor->localScore)
            {
                currentSolution = neighbor;
                neighbor->parent = current;
                neighbor->localScore = possiblyNewScore;

                neighbor->globalScore = neighbor->localScore + dist(currentLocation, endLocation);
            }
        }
    }

public:
    AStarPathFinding()
    {
        sAppName = "AStarPathFinding";
    }

    ~AStarPathFinding()
    {
        delete[] cells;
    }

    bool OnUserCreate() override
    {

        cols = ScreenWidth() / blockSize;
        rows = ScreenHeight() / blockSize;
        length = cols * rows;

        cells = new Cell[length];

        startX = 10;
        startY = 10;

        endX = cols - 10;
        endY = rows - 10;

        reset();
        return true;
    }

    bool OnUserUpdate(float fElapsedTime) override
    {
        if (GetMouse(0).bPressed)
            reset();

        auto p = [&](int x, int y) { return y * cols + x; };

        for (int i = 0; i < 50; i++)
            if (!solved)
                astarStep();

        Clear(olc::WHITE);
        for (int y = 0; y < rows; y++)
        {
            for (int x = 0; x < cols; x++)
            {
                olc::Pixel col;

                switch (cells[p(x, y)].state)
                {

                case blocked:
                    col = olc::BLACK;
                    break;
                default:
                    col = olc::WHITE;
                    break;
                }

                FillRect(x * blockSize, y * blockSize, blockSize, blockSize, col);
            }
        }

        FillCircle(startX * blockSize, startY * blockSize, blockSize * 1.5f, olc::RED);
        FillCircle(endX * blockSize, endY * blockSize, blockSize * 1.5f, olc::GREEN);

        if (currentSolution != NULL)
        {
            Cell *c = currentSolution;
            while (c->parent != NULL)
            {
                int thickness = 6;
                int fromX = c->x * blockSize + (blockSize / 2) - thickness / 2;
                int fromY = c->y * blockSize + (blockSize / 2) - thickness / 2;

                int toX = c->parent->x * blockSize + (blockSize / 2) - thickness / 2;
                int toY = c->parent->y * blockSize + (blockSize / 2) - thickness / 2;

                for (int i = 0; i < thickness; i++)
                {
                    DrawLine(fromX + i, fromY + i, toX + i, toY + i, olc::RED);
                }

                c = c->parent;
            }
        }

        return !GetKey(olc::Key::ESCAPE).bPressed;
    }
};

int main(int, char **)
{
    AStarPathFinding AStarPathFinding;
    if (AStarPathFinding.Construct(1900, 1600, 1, 1))
        AStarPathFinding.Start();

    return 0;
}
