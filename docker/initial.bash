ROOT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )/.." && pwd )"
cd

mkdir -p $ROOT_DIR/workspace/raisim_workspace
mkdir -p $ROOT_DIR/workspace/raisim_build

cd $ROOT_DIR/workspace/raisim_workspace

git clone https://github.com/raisimTech/raisimLib.git &&
git clone https://github.com/leggedrobotics/ogre.git &&
git clone https://github.com/raisimTech/raisimOgre.git &&
git clone https://github.com/pybind/pybind11.git

cd

cp -r $ROOT_DIR/docker/req/env $ROOT_DIR/workspace/raisim_workspace/raisimLib/raisimGymTorch/raisimGymTorch/

cp $ROOT_DIR/docker/req/CMakeLists.txt $ROOT_DIR/workspace/raisim_workspace/raisimLib/raisimGymTorch/

cp $ROOT_DIR/docker/req/OgreGLXGLSupport.cpp $ROOT_DIR/workspace/raisim_workspace/ogre/RenderSystems/GLSupport/src/GLX

cp -r /home/$USER/.raisim $ROOT_DIR/workspace/

cp $ROOT_DIR/docker/req/build.bash $ROOT_DIR/workspace/raisim_workspace/
cp $ROOT_DIR/docker/req/rebuild.bash $ROOT_DIR/workspace/raisim_workspace/

cd

