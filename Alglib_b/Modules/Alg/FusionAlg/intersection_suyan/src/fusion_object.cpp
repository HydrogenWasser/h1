#include <iostream>
#include "fusion_object.h"
#include <xtensor-blas/xlinalg.hpp>
#include <xtensor/xsort.hpp>


// TypeFusion::TypeFusion(xt::xarray<float> bbox){

//         framework = {'car': 0, 'bicycle': 1, 'bus': 2, 'motorbike': 3, 'person': 4,
//                           'semi': 5, 'truck': 6, 'jtruck': 7, 'Minibus': 8, 'road block': 9, 'constrution': 10,
//                           'carton': 11, 'unknown': 12};
//         mass_A = xt::zeros<float>({framework.size()});
//         fusion_label = bbox[7];
//         fusion_score = bbox[10];
//         _bbox = bbox;
//         mass_A = xt::full_like(mass_A,1-fusion_score);
//         // xt::view(mass_A,xt,all())=
//         mass_A[int(fusion_label)] = 1;
//         mass_A[int(mass_A.size()-1)] = 0;
//         int src = 2;
//         float dis_thresh = 120;
//         float intersection_center[2]={0,0};
//         _dis_thresh = dis_thresh;
//         _intersection_center = intersection_center;
    
// }
TypeFusion::TypeFusion(xt::xarray<float> bbox){

        _framework = {{"car",0},{"bicycle",1},{"bus",2},{"motorbike",3},{"person",4},{"semi",5},
        {"truck",6},{"jtruck",7},{"Minibus",8},{"road block",9},{"constrution",10},{"carton",11},
        {"unknown",12}
        };                 
        // framework = {{"car",0}, "bicycle": 1, "bus": 2, "motorbike": 3, "person": 4,
        //                   "semi": 5, "truck": 6, "jtruck": 7, "Minibus": 8, "road block": 9, "constrution": 10,
        //                   "carton": 11, "unknown": 12};
        mass_A = xt::zeros<float>({int(_framework.size())});

        fusion_label = bbox[7];
        fusion_score = bbox[10];
        if (fusion_score >= 0.98){
            fusion_score = 0.98;
        }
        _bbox = bbox;
        mass_A = xt::full_like(mass_A,1-fusion_score);
        // xt::view(mass_A,xt,all())=
        mass_A[int(fusion_label)] = 1;
        mass_A[int(mass_A.size()-1)] = 0;
        int src = 2;
        float dis_thresh = 120;
        float intersection_center[2]={0,0};
        _dis_thresh = dis_thresh;
        memcpy(&_intersection_center[0], &intersection_center[0], 2*sizeof(float));
    
}

void TypeFusion::updateType(xt::xarray<float> bbox, int src){
    float sensor_reduction_ratio;
    float dis_thresh;
    float intersection_center[2];
    float detect_score;

    if (src == 2){
        sensor_reduction_ratio = 0.5;
    }
    else if (src == 1){
        sensor_reduction_ratio = 1;
    }
    else{
        sensor_reduction_ratio = 0.5;
    }
    dis_thresh = _dis_thresh;
    // memcpy(&_intersection_center[0], &intersection_center[0], 2*sizeof(float));
    detect_score = bbox[10];
    detect_score *= sensor_reduction_ratio;
    // float dis = std::sqrt(std::pow(bbox[0]-intersection_center[0], 2) + std::pow(bbox[1]-intersection_center[1], 2));
    // float dis_pro = std::pow(2, dis / dis_thresh);
    // float label_score_pro = std::exp(-1 * dis_pro);
    // detect_score *= label_score_pro;
    bbox[10] = detect_score;
    mass_B = xt::zeros<float>({int(_framework.size())});
    mass_B = xt::full_like(mass_B,1-bbox[10]);
    // std::cout<<"bbox"<<bbox<<std::endl;
    // std::cout<<int(bbox[7])<<"bbox[10]"<<bbox[10]<<std::endl;
    // std::cout<<int(bbox[7])<<"zly-fusion_score"<<fusion_score<<std::endl;
    // std::cout<<mass_B<<std::endl;
    mass_B[int(bbox[7])] = 1;
    mass_B[int(mass_B.size()-1)] = 0;
    xt::xarray<float> T;
    float fenmu_A;
    float fenmu_B;
    xt::xarray<float> fenzi_A;
    xt::xarray<float> fenzi_B;
    xt::xarray<float> bayes_mass_A;
    xt::xarray<float> bayes_mass_B;
    xt::xarray<float> k;
    float total_k;
    xt::xarray<float> new_total;
    T = {1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 12};
    // T = np.array([1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 12])
    fenmu_A = xt::linalg::dot(mass_A, T)[0];
    fenmu_B = xt::linalg::dot(mass_B, T)[0];
    fenzi_A = mass_A*T;
    fenzi_B = mass_B*T;
    fenzi_A[int(fenzi_A.size())-1] = 0;
    fenzi_B[int(fenzi_B.size())-1] = 0;
    bayes_mass_A = (fenzi_A / fenmu_A);
    bayes_mass_B = (fenzi_B / fenmu_B);
    k = bayes_mass_A*bayes_mass_B;
    k[int(k.size())-1] = 0;
    total_k = xt::sum(k)[0];
    xt::xarray<float> new_label_score;
    new_total = bayes_mass_A*bayes_mass_B;
    new_label_score = new_total / total_k;
    mass_A = new_label_score;
    float new_score;
    int new_label;
    new_score = xt::amax(new_label_score)[0];
    new_label = int(xt::argmax(new_label_score)[0]);
    // std::cout<<"fusion_score:"<<fusion_score<<"new_score:"<<new_score<<std::endl;

    if (new_score >= 1){
        new_score = 0.98;
        mass_A[new_label] = 0.98;
        mass_A[int(mass_A.size()-1)] = 0;

    }
    // if((new_score-fusion_score)>0.1){
    //     fusion_label = new_label;
    // }
    
    fusion_score = new_score;
    fusion_label = new_label;
    

}