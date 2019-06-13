//
//  TransformObject.cpp
//  terrain2
//
//  Created by Derek Ortega on 5/12/19.
//


#include "TransformObject.h"

//  Base class for any object that needs a transform.
//
TransformObject::TransformObject() {
    position = ofVec3f(0, 0, 0);
    scale = ofVec3f(1, 1, 1);
    rotation = 0;
}

void TransformObject::setPosition(const ofVec3f & pos) {
    position = pos;
}
