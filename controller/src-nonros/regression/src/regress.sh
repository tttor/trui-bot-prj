#!/bin/bash
# if [ "$#" -ne 1 ]; then
#     echo "Illegal number of parameters"
#     exit 1
# fi

root_dir=/home/deanzaka/Github/trui-bot-prj/controller/src-nonros/regression
timestamp="$(date +'%Y%m%d.%H%M%S')"

#: main: (1) NuSVR and (2) GradientBoostingRegressor, 
#: secondary: GP (GaussianProcess), Lasso, DecisionTreeRegressionwithAdaBoost,
# method=$1
method=NuSVR
hyperparam_filename=v00
n_dataset_clone=1
dataset_name=rbmt-data
dataset_timestamp=20151904.1651

data_dirpath=$root_dir/data/$dataset_name.$dataset_timestamp
hyperparam_filepath=$root_dir/meta/$method-hyperparam/$hyperparam_filename.json

result_dirpath=$root_dir/regression-result/$dataset_name-$method-$timestamp
meta_filepath=$result_dirpath/regression.meta
mkdir -p $result_dirpath

#
exe=$root_dir/src/regressing.py
python  $exe \
		$data_dirpath \
		$result_dirpath \
		$meta_filepath \
		$method \
		$hyperparam_filepath \
		$n_dataset_clone

#
echo $data_dirpath >> $meta_filepath
echo $dataset_name >> $meta_filepath 
echo $annotator_id >> $meta_filepath
echo $hyperparam_filepath >> $meta_filepath 
echo $n_dataset_clone >> $meta_filepath
echo $timestamp >> $meta_filepath 
echo "$(date +'%Y%m%d.%H%M%S')" >> $meta_filepath 
