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
#include <global_planner/gradient_path.h>
#include <algorithm>
#include <stdio.h>
#include <global_planner/planner_core.h>

namespace global_planner {

GradientPath::GradientPath(PotentialCalculator* p_calc) : Traceback(p_calc), pathStep_(0.5)
{
    gradx_ = grady_ = nullptr;
}

GradientPath::~GradientPath()
{
    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
}

void GradientPath::setSize(int xs, int ys)
{
    Traceback::setSize(xs, ys);
    if (gradx_)
        delete[] gradx_;
    if (grady_)
        delete[] grady_;
    gradx_ = new float[static_cast<unsigned long>(xs * ys)];
    grady_ = new float[static_cast<unsigned long>(xs * ys)];
}

bool GradientPath::getPath(float* potential, double start_x, double start_y, double goal_x, double goal_y, std::vector<std::pair<float, float> >& path)
{
    std::pair<float, float> current;
    int stc = getIndex(static_cast<int>(goal_x), static_cast<int>(goal_y));

    // set up offset
    float dx = goal_x - static_cast<int>(goal_x);
    float dy = goal_y - static_cast<int>(goal_y);
    int ns = xs_ * ys_;
    memset(gradx_, 0, ns * sizeof(float));
    memset(grady_, 0, ns * sizeof(float));

    int c = 0;
    while (c++<ns*4)
    {
        // check if near goal
        double nx = stc % xs_ + dx, ny = stc / xs_ + dy;

        if (fabs(nx - start_x) < 0.5 && fabs(ny - start_y) < 0.5)
        {
            current.first = start_x;
            current.second = start_y;
            path.push_back(current);
            return true;
        }

        if (stc < xs_ || stc > xs_ * ys_ - xs_) // would be out of bounds
        {
            std::cout << "[PathCalc] Out of bounds" << std::endl;
            return false;
        }

        current.first = nx;
        current.second = ny;

        path.push_back(current);

        bool oscillation_detected = false;
        int npath = path.size();
        if (npath > 2 && fabs(path[npath - 1].first - path[npath - 3].first) < 0.01
                      && fabs(path[npath - 1].second - path[npath - 3].second) < 0.01)
        {
            oscillation_detected = true;
        }

        int stcnx = stc + xs_;
        int stcpx = stc - xs_;

        // check for potentials at eight positions near cell
        if (potential[stc] >= POT_HIGH || potential[stc + 1] >= POT_HIGH || potential[stc - 1] >= POT_HIGH
            || potential[stcnx] >= POT_HIGH || potential[stcnx + 1] >= POT_HIGH || potential[stcnx - 1] >= POT_HIGH
            || potential[stcpx] >= POT_HIGH || potential[stcpx + 1] >= POT_HIGH || potential[stcpx - 1] >= POT_HIGH
            || oscillation_detected)
        { //8������������һ��Ϊ�ϰ���
            //ROS_DEBUG("[Path] Pot fn boundary, following grid (%0.1f/%d)", potential[stc], (int) path.size());
            // check eight neighbors to find the lowest

          int minc = stc;
          int minp = potential[stc];
          int st = stcpx - 1;
          if (potential[st] < minp)
          {
              minp = potential[st];
              minc = st;
          }
          st++;
          if (potential[st] < minp)
          {
              minp = potential[st];
              minc = st;
          }
          st++;
          if (potential[st] < minp)
          {
              minp = potential[st];
              minc = st;
          }
          st = stc - 1;
          if (potential[st] < minp)
          {
              minp = potential[st];
              minc = st;
          }
          st = stc + 1;
          if (potential[st] < minp)
          {
              minp = potential[st];
              minc = st;
          }
          st = stcnx - 1;
          if (potential[st] < minp)
          {
              minp = potential[st];
              minc = st;
          }
          st++;
          if (potential[st] < minp)
          {
              minp = potential[st];
              minc = st;
          }
          st++;
          if (potential[st] < minp)
          {
              minp = potential[st];
              minc = st;
          }

          stc = minc;
          dx = 0;
          dy = 0;

          if (potential[stc] >= POT_HIGH)
          {
              return false;
          }
        }
        else // have a good gradient here
        {
            // get grad at four positions near cell
            gradCell(potential, stc);
            gradCell(potential, stc + 1);
            gradCell(potential, stcnx);
            gradCell(potential, stcnx + 1);

            // get interpolated gradient
            float x1 = (1.0 - dx) * gradx_[stc] + dx * gradx_[stc + 1];
            float x2 = (1.0 - dx) * gradx_[stcnx] + dx * gradx_[stcnx + 1];
            float x = (1.0 - dy) * x1 + dy * x2; // interpolated x
            float y1 = (1.0 - dx) * grady_[stc] + dx * grady_[stc + 1];
            float y2 = (1.0 - dx) * grady_[stcnx] + dx * grady_[stcnx + 1];
            float y = (1.0 - dy) * y1 + dy * y2; // interpolated y

            // show gradients
            //ROS_DEBUG("[Path] %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f  %0.2f,%0.2f; final x=%.3f, y=%.3f\n", gradx_[stc], grady_[stc], gradx_[stc+1], grady_[stc+1], gradx_[stcnx], grady_[stcnx], gradx_[stcnx+1], grady_[stcnx+1], x, y);

            // check for zero gradient, failed
            if (x == 0.0 && y == 0.0)
            {
                return false;
            }

            // move in the right direction
            float ss = pathStep_ / hypot(x, y);
            dx += x * ss;
            dy += y * ss;

            // check for overflow
            if (dx > 1.0)
            {
                stc++;
                dx -= 1.0;
            }
            if (dx < -1.0)
            {
                stc--;
                dx += 1.0;
            }
            if (dy > 1.0)
            {
                stc += xs_;
                dy -= 1.0;
            }
            if (dy < -1.0)
            {
                stc -= xs_;
                dy += 1.0;
            }
        }
    }
    return false;
}

//
// gradient calculations
//
// calculate gradient at a cell
// positive value are to the right and down
float GradientPath::gradCell(float* potential, int n)
{
    if (gradx_[n] + grady_[n] > 0.0)    // check this cell
        return 1.0;

    if (n < xs_ || n > xs_ * ys_ - xs_)    // would be out of bounds
        return 0.0;
    float cv = potential[n];
    float dx = 0.0;
    float dy = 0.0;

    // check for in an obstacle
    if (cv >= POT_HIGH)
    {
        if (potential[n - 1] < POT_HIGH)
            dx = -lethal_cost_;
        else if (potential[n + 1] < POT_HIGH)
            dx = lethal_cost_;

        if (potential[n - xs_] < POT_HIGH)
            dy = -lethal_cost_;
        else if (potential[n + xs_] < POT_HIGH)
            dy = lethal_cost_;
    }
    else                // not in an obstacle
    {
        // dx calc, average to sides
        if (potential[n - 1] < POT_HIGH)
            dx += potential[n - 1] - cv;
        if (potential[n + 1] < POT_HIGH)
            dx += cv - potential[n + 1];

        // dy calc, average to sides
        if (potential[n - xs_] < POT_HIGH)
            dy += potential[n - xs_] - cv;
        if (potential[n + xs_] < POT_HIGH)
            dy += cv - potential[n + xs_];
    }
    // normalize
    float norm = hypot(dx, dy);
    if (norm > 0)
    {
        norm = 1.0 / norm;
        gradx_[n] = norm * dx;
        grady_[n] = norm * dy;
    }
    return norm;
}

} //end namespace global_planner
