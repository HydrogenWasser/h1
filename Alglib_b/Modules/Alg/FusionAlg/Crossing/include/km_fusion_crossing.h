//
// Created by root on 4/14/21.
//


#include <xtensor/xarray.hpp>
//#include "CommonDef.h"
#ifndef SECOND_DETECTOR_KM_H
#define SECOND_DETECTOR_KM_H
#define zero_threshold 0.000001
#define match_val -7522752
#define minz 9900000000


namespace km_fusion_crossing {
    class KMNode {
    public:
        KMNode(int id, float exception = 0, int match = match_val, bool visit = false);


        int _id;
        float _exception;
        int _match;
        bool _visit;
    };

    class KuhnMunkres{
    public:
        KuhnMunkres();
        ~KuhnMunkres();
        void set_matrix(xt::xarray<float> &x_y_values);
        bool km_fusion();
        bool dfs(int i);
        void set_false(std::vector<km_fusion_crossing::KMNode> &nodes);
        void change_exception(std::vector<km_fusion_crossing::KMNode> &nodes, float &change);
        xt::xarray<int> get_connect_result();
        float get_max_value_result();

    private:
        xt::xarray<float> _matrix;
        std::vector<km_fusion_crossing::KMNode> _xnodes;
        std::vector<km_fusion_crossing::KMNode> _ynodes;
        float _minz = -100000000;
        float _x_length = 0;
        float _y_length = 0;
        int _index_x = 0;
        int _index_y;
    };

}
#endif //SECOND_DETECTOR_KM_H
