extern C {
    /*arParamrChangeSizeWrapper(int size, int width, int height) {
        arParamChangeSize(size, width, height);
    }*/

    int arParamDispWrapper(const ARParam *param) {
        return arParamDisp(param);
    };
}