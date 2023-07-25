cd


mkdir -p /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_workspace
mkdir -p /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_build

cd /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_workspace

git clone https://github.com/raisimTech/raisimLib.git &&
git clone https://github.com/leggedrobotics/ogre.git &&
git clone https://github.com/raisimTech/raisimOgre.git &&
git clone https://github.com/pybind/pybind11.git

cd

cp -r /home/$USER/code-2023-study-quadruped-gait/docker/req/env /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/

cp /home/$USER/code-2023-study-quadruped-gait/docker/req/CMakeLists.txt /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_workspace/raisimLib/raisimGymTorch/

cp /home/$USER/code-2023-study-quadruped-gait/docker/req/OgreGLXGLSupport.cpp /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_workspace/ogre/RenderSystems/GLSupport/src/GLX

cp -r /home/$USER/.raisim /home/$USER/code-2023-study-quadruped-gait/workspace/

cp /home/$USER/code-2023-study-quadruped-gait/docker/req/build.bash /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_workspace/
cp /home/$USER/code-2023-study-quadruped-gait/docker/req/rebuild.bash /home/$USER/code-2023-study-quadruped-gait/workspace/raisim_workspace/

cd

