直線追従
ターミナルから
ypspur-coordinator　を起動
--------------------------------
別ターミナルで
$ roslaunch yamabico_pkg line_GL.launch

各paramについて
<param name="point_x_double" value="2.0"/>
<param name="point_y_double" value="0.0"/>
    追従する直線が通る点のx,y
<param name="line_theta_radian" value="0.785"/>
    追従する直線の傾き(rad)
<param name="goal_x_double" value="4.0"/>
<param name="goal_y_double" value="2.0"/>
    停止する位置のx,y
<param name="max_speed" value="0.4"/>
    速度


-----------------------------------------------------------------
カルガモ
ターミナルから
ypspur-coordinator　を起動
--------------------------------
別ターミナルで
$ roslaunch yamabico_pkg urg_node.launch
--------------------------------
さらに別ターミナルで
$ rosrun yamabico_pkg karugamo




※karugamo.launchはgazeboで動かすときに使用します