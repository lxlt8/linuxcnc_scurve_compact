/********************************************************************
*   Written by Grotius, alias Skynet.
*   michelwijnja@gmail.com
*
* Author: Michel Wijnja
* License: GPL Version 2
* System: Linux
*
* Copyright (c) 2024 All rights reserved.
********************************************************************/
#ifndef SUB_SEGMENT_H
#define SUB_SEGMENT_H

#include "emcmot_segment.h"
#include "arc.h"
#include "line3d.h"
#include "clothoid3d.h"

/* Copy emcmot_segment data to the clothoid lib segment struct.
 * The clothoid lib's arc-helix center.z is equal to start.z
 * In linuxcnc the center.z is equal to end.z.
 * In the function the z axis is alinged to use with linuxcnc.
 *
 */
static inline void copy_emcmot_segment_to_sub_segment(struct emcmot_segment *seg){

    // We need to know wich plane is active.
    // printf("plane: %d \n",seg->tag.fields[4]);

    double p0[3];
    p0[0]=seg->start.tran.x;
    p0[1]=seg->start.tran.y;
    p0[2]=seg->start.tran.z;

    double p1[3];
    p1[0]=seg->end.tran.x;
    p1[1]=seg->end.tran.y;
    p1[2]=seg->end.tran.z;

    // We need to align the arc-helix center.
    // Helix in clothoid lib used the same plane for p0, pc.
    // In linuxcnc this is different.
    double pc[3];
    if(seg->tag.fields[4]==170){ // G17 xy top plane.
        pc[0]=seg->center.x;
        pc[1]=seg->center.y;
        pc[2]=seg->start.tran.z;
    }
    if(seg->tag.fields[4]==180){ // G18 zx.
        pc[0]=seg->center.x;
        pc[1]=seg->start.tran.y;
        pc[2]=seg->center.z;
    }
    if(seg->tag.fields[4]==190){ // G19 yz.
        pc[0]=seg->start.tran.x;
        pc[1]=seg->center.y;
        pc[2]=seg->center.z;
    }

    double pn[3];
    pn[0]=seg->normal.x;
    pn[1]=seg->normal.y;
    pn[2]=seg->normal.z;

    int turn = seg->consistent_turn; // The clothoid lib uses a consistent turn value for arc-helix.

    if(seg->canon_motion_type==1 || seg->canon_motion_type==2 || seg->canon_motion_type==4 || seg->canon_motion_type==5 ){
        seg->subseg.segment_type=LINE;
        init_line(p0,p1,&seg->subseg,1);
    }
    if(seg->canon_motion_type==3 && seg->arc_helix_dir==dir_G2){
        int cw=1;
        seg->subseg.segment_type=ARC;
        init_arc(p0,p1,pc,pn,cw,turn,&seg->subseg,1);
    }
    if(seg->canon_motion_type==3 && seg->arc_helix_dir==dir_G3){
        int cw=0;
        seg->subseg.segment_type=ARC;
        init_arc(p0,p1,pc,pn,cw,turn,&seg->subseg,1);
    }
    if(seg->canon_motion_type==4){ // Tool change.
        seg->subseg.segment_type=LINE;
        init_line(p0,p1,&seg->subseg,1);
    }
    if(seg->canon_motion_type==10){
        seg->subseg.segment_type=CLOTHOID;
    }

    seg->length_netto=seg->subseg.length;
}

#endif // SUB_SEGMENT_H
