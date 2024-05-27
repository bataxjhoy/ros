/*********************************************************************
 *
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2008, 2013, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage, Inc. nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * Author: Eitan Marder-Eppstein
 *         David V. Lu!!
 *********************************************************************/
#include<global_planner/dijkstra.h>
#include <algorithm>
namespace global_planner {

DijkstraExpansion::DijkstraExpansion(PotentialCalculator* p_calc, int nx, int ny) :
        Expander(p_calc, nx, ny), pending_(NULL), precise_(false)
{
    // priority buffers
    buffer1_ = new int[PRIORITYBUFSIZE];
    buffer2_ = new int[PRIORITYBUFSIZE];
    buffer3_ = new int[PRIORITYBUFSIZE];

    priorityIncrement_ = 2 * neutral_cost_;
}

DijkstraExpansion::~DijkstraExpansion()
{
  delete[] buffer1_;
  delete[] buffer2_;
  delete[] buffer3_;
  if (pending_)
      delete[] pending_;
}

//
// Set/Reset map size
//
void DijkstraExpansion::setSize(int xs, int ys)
{
    Expander::setSize(xs, ys);
    if (pending_)
        delete[] pending_;

    pending_ = new bool[static_cast<unsigned long>(ns_)];
    memset(pending_, 0, static_cast<unsigned long>(ns_) * sizeof(bool));
}

//
// main propagation function
// Dijkstra method, breadth-first
// runs for a specified number of cycles,
//   or until it runs out of cells to update,
//   or until the Start cell is found (atStart = true)
//

bool DijkstraExpansion::calculatePotentials(unsigned char* costs,
                                            double start_x, double start_y,
                                            double end_x, double end_y,
                                            int cycles,
                                            float* potential)
{
    cells_visited_ = 0;
    // priority buffers
    threshold_ = lethal_cost_;
    currentBuffer_ = buffer1_;
    currentEnd_ = 0;
    nextBuffer_ = buffer2_;
    nextEnd_ = 0;
    overBuffer_ = buffer3_;
    overEnd_ = 0;
    memset(pending_, 0, static_cast<unsigned long>(ns_) * sizeof(bool));
    std::fill(potential, potential + ns_, POT_HIGH);

    // set goal
    int k = toIndex(static_cast<int>(start_x), static_cast<int>(start_y));

    if(precise_)
    {
        double dx = start_x - static_cast<int>(start_x), dy = start_y - static_cast<int>(start_y);
        dx = floorf(dx * 100 + 0.5) / 100;
        dy = floorf(dy * 100 + 0.5) / 100;
        potential[k] = neutral_cost_ * 2 * dx * dy;
        potential[k+1] = neutral_cost_ * 2 * (1-dx)*dy;
        potential[k+nx_] = neutral_cost_*2*dx*(1-dy);
        potential[k+nx_+1] = neutral_cost_*2*(1-dx)*(1-dy);

        push_cur(k+2);
        push_cur(k-1);
        push_cur(k+nx_-1);
        push_cur(k+nx_+2);

        push_cur(k-nx_);
        push_cur(k-nx_+1);
        push_cur(k+nx_*2);
        push_cur(k+nx_*2+1);
    }
    else
    {
        potential[k] = 0;
        push_cur(k+1);
        push_cur(k-1);
        push_cur(k-nx_);
        push_cur(k+nx_);
    }

    int nwv = 0;            // max priority block size
    int nc = 0;            // number of cells put into priority blocks
    int cycle = 0;        // which cycle we're on

    // set up start cell
    int startCell = toIndex(end_x, end_y);

    for (; cycle < cycles; cycle++) // go for this many cycles, unless interrupted
    {
        if (currentEnd_ == 0 && nextEnd_ == 0) // priority blocks empty
            return false;
        // stats
        nc += currentEnd_;
        if (currentEnd_ > nwv)
            nwv = currentEnd_;

        // reset pending_ flags on current priority buffer
        int *pb = currentBuffer_;
        int i = currentEnd_;
        while (i-- > 0)
            pending_[*(pb++)] = false;

       
         //以间隔15邻域探索
        for(int j = 30; j > 17; j = j - 3)
        {
            // process current priority buffer
            pb = currentBuffer_;
            i = currentEnd_;
            while (i-- > 0)
                updateCell(j, costs, potential, *pb++);
            //ROS_WARN("updateCell j= %d", j);
            if(nextEnd_ != 0)break;                
        }
 
        // swap priority blocks currentBuffer_ <=> nextBuffer_
        currentEnd_ = nextEnd_;
        nextEnd_ = 0;
        pb = currentBuffer_;        // swap buffers
        currentBuffer_ = nextBuffer_;
        nextBuffer_ = pb;

        // see if we're done with this priority level
        if (currentEnd_ == 0)
        {
            threshold_ += priorityIncrement_;    // increment priority threshold
            currentEnd_ = overEnd_;    // set current to overflow block
            overEnd_ = 0;
            pb = currentBuffer_;        // swap buffers
            currentBuffer_ = overBuffer_;
            overBuffer_ = pb;
        }

        // check if we've hit the Start cell
        if (potential[startCell] < POT_HIGH)
            break;
    }
    if (cycle < cycles)
        return true; // finished up here
    else
        return false;
}

//
// Critical function: calculate updated potential value of a cell,
//   given its neighbors' values
// Planar-wave update calculation from two lowest neighbors in a 4-grid
// Quadratic approximation to the interpolated value
// No checking of bounds here, this function should be fast
//

#define INVSQRT2 0.707106781

inline void DijkstraExpansion::updateCell( int temp,unsigned char* costs, float* potential, int n)
{
    cells_visited_++;//已经遍历过的栅格 自加1

    // do planar wave update 做平面波更新
    float c = getCost(costs, n);
    if (c >= lethal_cost_)    // //大于致命代价,不要启发到障碍里面去
        return;

    float c_near[8] = {0};
    if(n>temp*(nx_+1) && n < nx_*ny_-temp*(nx_+1))
    {
        int np = n - temp*nx_;
        int nn = n + temp*nx_;
        c_near[0] = getCost(costs, np-temp);
        c_near[1] = getCost(costs, np);
        c_near[2] = getCost(costs, np+temp);

        c_near[3] = getCost(costs, n-temp);
        c_near[4] = getCost(costs, n+temp);

        c_near[5] = getCost(costs, nn-temp);
        c_near[6] = getCost(costs, nn);
        c_near[7] = getCost(costs, nn+temp);

        for(unsigned int i=0;i<8;i++)
        {
            if(c_near[i] > 50)
            {
                return;
            }
        }
    }

    float pot = p_calc_->calculatePotential(potential, c, n); //pot  =  前后左右最小的potential + 当前的costs 

    // now add affected neighbors to priority blocks现在将受影响的邻居添加到优先级块中
    if (pot < potential[n])
    {
        float le = INVSQRT2 * (float)getCost(costs, n - 1);  //获得上下左右的 cost值, 但是不明白为啥要乘以二分之根号二
        float re = INVSQRT2 * (float)getCost(costs, n + 1);
        float ue = INVSQRT2 * (float)getCost(costs, n - nx_);
        float de = INVSQRT2 * (float)getCost(costs, n + nx_);
        potential[n] = pot;
        //ROS_INFO("UPDATE %d %d %d %f", n, n%nx, n/nx, potential[n]);
        if (pot < threshold_)    // low-cost buffer block //如果当前 pot小于阈值
        {
            if (potential[n - 1] > pot + le)  //如果当前点的潜力  大于 潜力-1 + cost*根号二/2  //将其push到next的队列
                push_next(n-1);
            if (potential[n + 1] > pot + re)
                push_next(n+1);
            if (potential[n - nx_] > pot + ue)
                push_next(n-nx_);
            if (potential[n + nx_] > pot + de)
                push_next(n+nx_);
        }
        else            // overflow block
        {
            if (potential[n - 1] > pot + le)  //如果当前点的潜力  大于 潜力-1 + cost*根号二/2  //将其push到over的队列
                push_over(n-1);
            if (potential[n + 1] > pot + re)
                push_over(n+1);
            if (potential[n - nx_] > pot + ue)
                push_over(n-nx_);
            if (potential[n + nx_] > pot + de)
                push_over(n+nx_);
        }
    }
}
} //end namespace global_planner
