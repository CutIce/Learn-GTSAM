//
// Created by hsh on 2021/10/20.
//

#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/PriorFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/nonlinear/Marginals.h>


#include <gtsam/nonlinear/Values.h>
#include <gtsam/base/Vector.h>
#include <CppUnitLite/TestHarness.h>
#include <boost/shared_ptr.hpp>
#include <boost/assign/std/list.hpp>


#include <iostream>

using namespace std;
using namespace gtsam;

int main(int argc, char ** argv) {
    NonlinearFactorGraph graph;
    Pose2 priorMean(0, 0, 0);

    noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::sigmas(Vector3(0.3, 0.3, 0.1));
    noiseModel::Diagonal::shared_ptr odomotryNoise = noiseModel::Diagonal::Sigmas(Vector3(0.2, 0.2, 0.1));

    graph.emplace_shared<PriorFactor<Pose2>>(1, priorMean, priorNoise);

    Pose2 odometry(2.0, 0.0, 0.0);
    graph.emplace_shared<BetweenFactor<Pose2>>(1, 2, odomotry, odomotryNoise);
    graph.emplace_shared<BetweenFactor<Pose2>>(2, 3, odomotry, odomotryNoise);

    graph.print("\nFactor Graph:\n");

    Values initial;
    initial.insert(1, Pose2(0.5, 0.0, 0.2));
    initial.insert(2, Pose2(2.3, 0.1, -0.2));
    initial.insert(3, Pose2(4.1, 0.1, 0.1));
    initial.print("\nInitial Estimate:\n");

    Values result = LevenbergMarquardtOptimizer(graph, initial).optimize();
    result.print("\nFinal Result:\n");

    cout.precision(2);


    Marginals marginals(graph, result);

    cout << endl;
    cout << "    x1 covar: \n" << marginals.marginalCovariance(1) << endl;
    cout << "    x2 covar: \n" << marginals.marginalCovariance(2) << endl;
    cout << "    x3 covar: \n" << marginals.marginalCovariance(3) << endl;
    cout << "    x1 marginalInformation:\n" << marginals.marginalInformation(1) << endl;
    cout << "    x2 marginalInformation:\n" << marginals.marginalInformation(2) << endl;
    cout << "    x3 marginalInformation:\n" << marginals.marginalInformation(3) << endl;


    for (const gtsam::Values::ConstKeyValuePair& key_value: result) {
        cout << "  key  " << key_value.key << endl;
        Pose2 tmp_pose = key_value.value.cast<gtsam::Pose2>();
        cout << "  data  " << tmp_pose.matrix() << endl;
    }
    for ( gtsam::NonlinearFactor::shared_ptr factor: graph ) { //遍历因子图节点
        gtsam::BetweenFactor<gtsam::Pose2>::shared_ptr f = dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose2>>( factor );//提取边
        if (f){
            gtsam::SharedNoiseModel model = f->noiseModel();//提取噪声模型
            gtsam::noiseModel::Diagonal::shared_ptr DiagModel = dynamic_pointer_cast<gtsam::noiseModel::Diagonal>( model );//从噪声模型提取高斯模型
            if ( DiagModel ) {
                gtsam::Matrix info;
                gtsam::Vector DiagModel_sigma=DiagModel->sigmas();
                cout<<"DiagModel_sigma:"<<DiagModel_sigma<<endl;//输出误差
                Pose2 pose=f->measured();
                cout<<"data:"<<pose.matrix()<<endl;//显示测量
            }
        }
        }
    return 0;
}