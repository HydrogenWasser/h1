//
// Created by root on 4/14/21.
//

#include "km_fusion_crossing.h"
#include <xtensor/xview.hpp>
#include <xtensor/xrandom.hpp>
#include <xtensor/xmath.hpp>
#include <xtensor/xadapt.hpp>
#include <xtensor/xstrides.hpp>
#include <xtensor-blas/xlinalg.hpp>

km_fusion_crossing::KMNode::KMNode(int id, float exception, int match, bool visit):
                    _id(id),
                    _exception(exception),
                    _match(match),
                    _visit(visit){

}

km_fusion_crossing::KuhnMunkres::KuhnMunkres() {}

km_fusion_crossing::KuhnMunkres::~KuhnMunkres() {}

void km_fusion_crossing::KuhnMunkres::set_matrix(xt::xarray<float> &x_y_values) {
    _x_length = x_y_values.shape(0);
    _y_length = x_y_values.shape(1);
    if (_x_length < _y_length){
        _index_x = 0;
        _index_y = 1;
    } else{
        _index_x = 1;
        _index_y = 0;
        x_y_values = xt::transpose(x_y_values);
        std::swap(_x_length, _y_length);
    }

    for (int i = 0; i < _x_length; ++i) {
        _xnodes.push_back(km_fusion_crossing::KMNode(i));
    }

    for (int j = 0; j < _y_length; ++j) {
        _ynodes.push_back(km_fusion_crossing::KMNode(j));
    }

    _matrix = x_y_values;
    for (int k = 0; k < _x_length; ++k) {
        _xnodes[k]._exception = xt::amax(xt::view(_matrix, k), 0)(0);
    }
}

void km_fusion_crossing::KuhnMunkres::set_false(std::vector<km_fusion_crossing::KMNode> &nodes) {
    for (int i = 0; i < nodes.size(); ++i) {
        nodes[i]._visit = false;
    }
}


bool km_fusion_crossing::KuhnMunkres::dfs(int i) {
    _xnodes[i]._visit = true;
    for (int j = 0; j < _y_length; ++j) {
        if (!_ynodes[j]._visit){
            double t = double (_xnodes[i]._exception + _ynodes[j]._exception - _matrix(i, j));
            t = std::floor(double (t *  10000000 + 0.5)) / 10000000;
            if (std::abs(t) < zero_threshold){
                _ynodes[j]._visit = true;
//                bool aaaa = dfs(_ynodes[j]._match);
                if (_ynodes[j]._match == match_val || dfs(_ynodes[j]._match)){
                    _xnodes[i]._match = j;
                    _ynodes[j]._match = i;
                    return true;
                }
            } else{
                if (t >= zero_threshold)
                    _minz = float (std::min(double (_minz), t));
            }
        }
    }
    return false;
}

void km_fusion_crossing::KuhnMunkres::change_exception(std::vector<km_fusion_crossing::KMNode> &nodes, float &change) {
    for (int i = 0; i < nodes.size(); ++i) {
        if (nodes[i]._visit)
            nodes[i]._exception += change;
    }
}
int nMax1 = 0;
bool  km_fusion_crossing::KuhnMunkres::km_fusion() {
        int nCount = 0;
    for (int i = 0; i < _x_length; ++i) {
        nCount = 0;
        while (true){
            nCount++;
            if(nCount > 200) 
            {
                return false;
            }
            _minz = minz;
            set_false(_xnodes);
            set_false(_ynodes);
            if (dfs(i))
                break;
            auto change = -1 * _minz;
            change_exception(_xnodes, change);
            change_exception(_ynodes, _minz);
        }
        if(nCount >= nMax1) 
            nMax1 = nCount;
    }
   
//    LOG(ERROR) << " nCount  = "<< nCount << " nMax = "<<nMax1;
    return true;
}

xt::xarray<int> km_fusion_crossing::KuhnMunkres::get_connect_result() {
    xt::xarray<int> ret = xt::xarray<int>({{0, 0}});
    for (int i = 0; i < _x_length; ++i) {
        int j = _xnodes[i]._match;
        int x_id = _xnodes[i]._id;
        int y_id = _ynodes[j]._id;
        if (_index_x == 1 and _index_y == 0)
            std::swap(x_id, y_id);
        ret = xt::concatenate(xt::xtuple(ret, xt::xarray<int>({{x_id, y_id}})),0);
    }
    return xt::view(ret, xt::range(1, ret.shape(0)));
}


float km_fusion_crossing::KuhnMunkres::get_max_value_result() {
    float ret = 0;
    for (int i = 0; i < _x_length; ++i) {
        int j = _xnodes[i]._match;
        ret += _matrix(i, j);
    }
    return ret;
}

