
/**************************************************************************
 * Copyright 2023 Youibot Robotics Co., Ltd. All Rights Reserved.
 * Contact: wenzhaochen (robotmanciu@gmail.com)
 * 
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <https://www.gnu.org/licenses/>.
 *************************************************************************/

#ifndef _COPPER_TAG_COMMON_H_
#define _COPPER_TAG_COMMON_H_

#include "opencv2/opencv.hpp"

// ANSI escape codes for text color
#define ANSI_COLOR_RESET   "\033[0m"
#define ANSI_COLOR_GREEN   "\033[32m"
#define ANSI_COLOR_YELLOW  "\033[33m"
#define ANSI_COLOR_RED     "\033[31m"

#if DEBUG
    #define DEBUG_INFO(message) do {\
        std::cout << ANSI_COLOR_RESET << "[" << __FUNCTION__ << "][" << __LINE__ << "]: " << ANSI_COLOR_GREEN << message << ANSI_COLOR_RESET << std::endl; \
    } while (0)
    #define DEBUG_WARNING(message) do {\
        std::cout << ANSI_COLOR_RESET << "[" << __FUNCTION__ << "][" << __LINE__ << "]: " << ANSI_COLOR_YELLOW << message << ANSI_COLOR_RESET << std::endl; \
    } while (0)
    #define DEBUG_ERROR(message) do {\
        std::cout << ANSI_COLOR_RESET << "[" << __FUNCTION__ << "][" << __LINE__ << "]: " << ANSI_COLOR_RED << message << ANSI_COLOR_RESET << std::endl; \
    } while (0)
#else
    #define DEBUG_INFO(message)
    #define DEBUG_WARNING(messge)
    #define DEBUG_ERROR(message)
#endif

#define DISDEBUG 0

/**
 * @brief Disjoint Set Union class for segments grouping 
 */
class DSU {
public:
    DSU(int n) : parent(n), size(n, 1) {
        for (int i = 0; i < n; ++i) {
            parent[i] = i;
        }
    }

    int find(int x) {
        if (parent[x] != x) {
            parent[x] = find(parent[x]);
        }
        return parent[x];
    }

    void unite(int x, int y) {
        int rootX = find(x);
        int rootY = find(y);
        
        if (rootX != rootY) {
            if (size[rootX] < size[rootY]) {
                std::swap(rootX, rootY);
            }
            parent[rootY] = rootX;
            size[rootX] += size[rootY];
        }
    }

    int getSize(int x) {
        return size[find(x)];
    }
private:
    std::vector<int> parent;
    std::vector<int> size;
}; // class DSU

/**
 * @brief reverse point(x,y) to point(y,x)
 * 
 * @param in 
 * @param out 
 */
template <typename T>
void reverse_xy(std::vector<std::vector<T>> &in, 
                std::vector<std::vector<T>> &out) {
    out.clear();
    for(int i = 0; i < in.size(); i++){
        std::vector<T> tmp;
        for(int j = 0; j < in[i].size(); j++){
            tmp.push_back(T(in[i][j].y, in[i][j].x));
        }
        out.push_back(tmp);
    }
}

/**
 * @brief convert any cv::Point type to cv::point for cv::polylines
 * 
 * @param points 
 * @return std::vector<cv::Point> 
 */
template <typename T>
std::vector<cv::Point> convert_to_integer(const std::vector<T>& points) {
    std::vector<cv::Point> intPoints;
    for (const auto& point : points) {
        intPoints.push_back(cv::Point((int)point.x, (int)point.y));
    }

    return intPoints;
}

/**
 * @brief cross product of 2 vectors; 
 * 
 * @param vec1 
 * @param vec2 
 * @return int 
 */
template <typename T>
int cross(T vec1, T vec2) {
    return vec1.x * vec2.y - vec1.y * vec2.x;
}

/**
 * @brief output line angle in deg for the line created by the startPt and endPt
 * 
 * @param startPt 
 * @param endPt 
 * @return double 
 */
template <typename T>
double line_angle(T startPt, T endPt) {
    double y = endPt.y - startPt.y;
    double x = endPt.x - startPt.x;
    return abs(std::atan(y / x) * 180.0 / M_PI + 90); 
}

template <typename T>
bool strict_in_line(T p1, T p2, T p3, double thresh = 10) {
    cv::Point vector1 = p2 - p1;
    cv::Point vector2 = p3 - p1;

    // Calculate the cross product of the two vectors
    int crossProduct = vector1.x * vector2.y - vector1.y * vector2.x;

    // If the cross product is close to zero, the points are considered collinear
    return std::abs(crossProduct) < thresh;
}

/**
 * @brief output line expression in pair<m, b> format
 * 
 * @param p1 
 * @param p2 
 * @return std::pair<double, double> 
 */
template <typename T>
std::pair<double, double> find_line_mb(T p1, T p2) {
    double m;
    if ((p2.x - p1.x) != 0) {
        m = (p2.y - p1.y) / (p2.x - p1.x);
    } else {
        m = 999;
    }
    double b = p1.y - m * p1.x;
    return std::make_pair(m, b); 
}

/**
 * @brief point dist to line
 * 
 * @param point 
 * @param linePt1 
 * @param linePt2 
 * @return double 
 */
template <typename T>
double point_dist_to_line(T point, T linePt1, T linePt2) {
    double a = linePt2.y - linePt1.y;
    double b = linePt1.x - linePt2.x;
    double c = linePt2.x * linePt1.y - linePt1.x * linePt2.y;

    return std::abs(a * point.x + b * point.y + c) / std::sqrt(a * a + b * b);
}

template <typename T>
double distance_of_2point(const T p1, const T p2) {
    return std::sqrt(std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2));
}

// Hash function for std::pair<int, int>
struct pairhash {
    template <class T1, class T2>
    std::size_t operator() (const std::pair<T1, T2>& p) const {
        // Combining the hashes of the two integers to get a single hash
        std::hash<T1> hash1;
        std::hash<T2> hash2;
        return hash1(p.first) << 1 + hash1(p.first) + hash2(p.second);
    }
};

#endif // _COPPER_TAG_COMMON_H_