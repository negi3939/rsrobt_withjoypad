# For servo mortor manipulator
サーボモータを用いたマニピュレータ用
joypadを用いて操作

# Inverse kinetmatics
mymath.cpp と　solvenu.cppと inversekinematics.cppを一緒に用いることで使える．DHパラメータをinvkSolvenu::setdhparameter()で指定し，目標手先位置姿勢(クオータニオン)をinvkSolvenu::settargetfx()で設定する．回転行列からクオータニオンを得るには invkSolvenu::matrixtoquatanion()を用いる．invkSolvenu::getangle()で設定した手先位置姿勢となる関節角度(-π<θ<π)を得る．
copyメソッドを所有しており,このクラスを継承すればDHパラメータ等をまるごとコピーできる．

# Inverse Dynamics
mymath.cpp と　solvenu.cppと inversekinematics.cppとinversdynamics.cppを一緒に用いることで使える．DHパラメータはinvdSolvenu::setdhparameter()で指定することもinvdSolvenu::copy()でinverse kinematics用のインスタンスを与えてしてすることもできる．
目標手先速度角速度をinvdSolvenu::settargetfx()で設定する．invkSolvenu::getangvel()で設定した手先速度角速度となる関節角角度を得る．

# Using Futabamotor manipulator 
Rsmotor.cppを用いてFutabaモータを用いた任意関節のアームを操作．Rsmotor::move(Vector3d x)で水平な姿勢の手先，Rsmotor::move(Vector3d x,double theta)で任意の姿勢の手先へ移動させる．
ファイルへ書き込むための配列が確保され，インスタンスがdeleteされる際にファイルへ書き出す

# Using joypad
Joypadxy.cppを用いてjoypadの操作に応じて値を加算減算する．別スレッドを立ち上げるのでメインスレッドは通信遅延に依存しない．三角を押すと終了する．


# Makefile
targetを指定するにはmake時にtargetに格納させて使う．また，基本的に実行入る作成後，そのまま実行させる使用にしているが，実行させたくない際はargv=0をつける．
実行ファイルに引数を付けたい場合はargv=hogeをつけると./hogehoge.out hogeのようにコマンド引数を渡せる．
projectを追加する際にはifeq()を用いてSOURCE_MAINとSOURCE_SUBにそれぞれファイルを指定する．
以下使用例
    $ make                     # You can get the executable file which written in TARGET. And the executable file will run.
    $ make target=hoge         # You can get the executable file which written in hoge. And the executable file will run.    
    $ make argv=hoge           # You can get the executable file which written in TARGET. And the executable file will run with hoge. 
    $ make notrun=1            # You can get the executable file which written in TARGET. The executable file will not run.	
    $ make clean               # The executable file which written in TARGET will removed.
    $ make clean target=hoge   # The executable file which written in hoge will removed.     

