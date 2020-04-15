/*
Created on Saturday Apr 13 2020
Author : yu du
Email : yuduseu@gmail.com
Last edit date :
*/
#pragma once

#ifndef BEHAVIOUR_H
#define BEHAVIOUR_H

#ifdef __cplusplus
extern "C" {
#endif

bool grasp_and_place(double* i_pos, double* size, double* o_pos);

bool grasp2_and_place(double* i_pos, double* size, double* o_pos);

#ifdef __cplusplus
}
#endif

#endif