/**************************************************************************/
/*                                                              /\/\      */
/*                                                              \  /      */
/*                                                              /  \      */
/* Author: Arnoud Visser, Joris van Dam, Casper Dik          _  \/\/  _   */
/*         University of Amsterdam                          | |      | |  */
/*         Dept. of Computer Systems                        | | /\/\ | |  */
/*         Kruislaan 403, NL 1098 SJ Amsterdam              | | \  / | |  */
/*         THE NETHERLANDS                                  | | /  \ | |  */
/*                                                          | | \/\/ | |  */
/*                                                          | \______/ |  */
/* This software has been written for the robot course       \________/   */
/* at our department. No representations are made about                   */
/* the suitability of this software for any purpose other       /\/\      */
/* than education.                                              \  /      */
/*                                                              /  \      */
/*                                                              \/\/      */
/* This file is copyright protected.                                      */
/* Copyright (c) 1991-2022 Universiteit van Amsterdam                     */
/* This software or any part thereof may only be used for non-commercial  */
/* research or purposes, as long as the author and University are         */
/* mentioned. Commercial use without explicit prior written consent by    */
/* the Universiteit van Amsterdam is strictly prohibited. This copyright  */
/* notice must be included with any copy of this software or any part     */
/* thereof.                                                               */
/*                                                                        */
/* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS    */
/* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT      */
/* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR  */
/* A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT   */
/* OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,  */
/* SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT       */
/* LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,  */
/* DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY  */
/* THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT    */
/* (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE  */
/* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.   */
/**************************************************************************/
/*
 * $Id$
 *
 * The calls to rtxdriver.
 * Simple copy of rtx.pas
 */
#include <stdio.h>
#include <cstdio>
#include <iostream>


#include "umi_rtx_controller/umi-drivers/ipcrt.h"
#include "umi_rtx_controller/umi-drivers/rtx.h"
#include "umi_rtx_controller/umi-drivers/comm.h"
#include "umi_rtx_controller/umi-drivers/armraw.h"
#include "umi_rtx_controller/umi-drivers/armlib.h"

using namespace std; 

static int arm_debug = 0;

int arm_raw_command(int ip,int cmdlen,int b1,int b2,int b3)
{
    libio rtxparams;

    rtxparams.params[0] = ip;
    rtxparams.params[1] = cmdlen;
    rtxparams.params[2] = b1;
    rtxparams.params[3] = b2;
    rtxparams.params[4] = b3;

    return arm_interrupt(RTX_CMD_RAW_CMD,&rtxparams);
}

int arm_raw_response(int ip,int *len,int *resp)
{
    libio rtxparams;
    int err;

    rtxparams.params[0] = ip;
    err = arm_interrupt(RTX_CMD_RAW_RESP,&rtxparams);
    *len = rtxparams.params[0];
    resp[0] = rtxparams.params[1];
    resp[1] = rtxparams.params[2];
    resp[2] = rtxparams.params[3];
    return err;
}

int arm_init_comms(int toggle,int debug)
{
    libio rtxparams;
    int err;

    arm_debug = debug;

    if(arm_debug>0)
       printf("======================= ARM version 24 March 2022\n");

    rtxparams.params[0] = toggle;
    rtxparams.params[1] = debug;
    err = arm_interrupt(RTX_CMD_INIT,&rtxparams);
    return err;
}

int arm_version(int version[])
{
    libio rtxparams;
    int err = arm_interrupt(RTX_CMD_VERSION,&rtxparams);

    if (err == 0) {
	version[0] = rtxparams.params[0];
	version[1] = rtxparams.params[1];
	version[2] = rtxparams.params[2];
    }
    return err;
}



int arm_restart()
{
    return arm_interrupt(RTX_CMD_RESTART,(libio *) 0);
}

int arm_define_origin()
{
    return arm_interrupt(RTX_CMD_DEFINE_HOME,(libio *) 0);
}

int arm_reload_pids()
{
    return arm_interrupt(RTX_CMD_RELOAD_PIDS,(libio *) 0);
}

int arm_set_mode(int motor,int mode)
{
    libio rtxparams;

    rtxparams.params[0] = motor;
    rtxparams.params[1] = mode;

    return arm_interrupt(RTX_CMD_SET_MODE,&rtxparams);
}

int arm_stop(int mode){
    libio rtxparams;


    if(arm_debug>0)
        printf("arm_stop(%d)\n",mode);

    rtxparams.params[0] = mode;
    rtxparams.params[1] = 2; /* AVI: added June 2014 */
    return arm_interrupt(RTX_CMD_STOP,&rtxparams);
}

int arm_go(int m,int bits)
{
    libio rtxparams;

    rtxparams.params[0] = m;
    rtxparams.params[1] = bits;
    return arm_interrupt(RTX_CMD_GO,&rtxparams);
}

int
arm_motor_status(int motor,int *status) {
    libio rtxparams;
    int err;

    rtxparams.params[0] = motor;
    err = arm_interrupt(RTX_CMD_MOTOR_STATUS,&rtxparams);
    *status = rtxparams.params[0];
    return err;
}

int
arm_general_status(int *status){
    libio rtxparams;
    int err;

    err = arm_interrupt(RTX_CMD_GENERAL_STATUS,&rtxparams);
    *status = rtxparams.params[0];
    return err;
}

int arm_read(int motor,int code,int *result)
{
    int err;
    libio rtxparams;

    rtxparams.params[0] = motor;
    rtxparams.params[1] = code;
    err = arm_interrupt(RTX_CMD_READ,&rtxparams);
    // cout << "arm_read :" << endl;
    *result = rtxparams.params[0];
    return err;
}

int arm_write(int motor,int code,int input){
    libio rtxparams;

    rtxparams.params[0] = motor;
    rtxparams.params[1] = code;
    rtxparams.params[2] = input;
    return arm_interrupt(RTX_CMD_WRITE,&rtxparams);
}

int arm_interpolate(int *data){
    libio rtxparams;
    int i;

    for (i = 0; i < ZEDOWN; i++)
	rtxparams.params[i] = data[i];

    return arm_interrupt(RTX_CMD_INTERPOLATE,&rtxparams);
}

int arm_soak(int s)
{
    libio rtxparams;

    rtxparams.params[0] = s;
    return arm_interrupt(RTX_CMD_SOAK,&rtxparams);
}

int arm_raw(int ip,int cmdlen,int b1,int b2,int b3,int *reslen,int *resp)
{
    libio rtxparams;
    int err;

    rtxparams.params[0] = ip;
    rtxparams.params[1] = cmdlen;
    rtxparams.params[2] = b1;
    rtxparams.params[3] = b2;
    rtxparams.params[4] = b3;

    err = arm_interrupt(RTX_CMD_RAW,&rtxparams);
    if (err == 0) {
	*reslen = rtxparams.params[0];
	resp[0] = rtxparams.params[1];
	resp[1] = rtxparams.params[2];
	resp[2] = rtxparams.params[3];
    }
    return err;
}
