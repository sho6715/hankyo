// 多重コンパイル抑止
#ifndef _MODE_H
#define _MODE_H

// *************************************************************************
//   ロボット名	： AEGIS
//   概要		： 
//   注意		： なし
//   メモ		： なし
// *************************************************************************

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <stdio.h>							// 標準入出力
#include <hal.h>							// HAL
#include <search.h>							// search


//**************************************************
// 定義（define）
//**************************************************

//**************************************************
// 列挙体（enum）
//**************************************************

//**************************************************
// 構造体（struct）
//**************************************************

//**************************************************
// グローバル変数
//**************************************************

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
PUBLIC BOOL MODE_DistRightCheck(void);
PUBLIC BOOL MODE_DistLeftCheck(void);
PUBLIC BOOL MODE_setWaitCheck(void);
PUBLIC BOOL MODE_CheckExe(void);
PUBLIC void LED_count(UCHAR number);

#endif