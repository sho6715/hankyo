// *************************************************************************
//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： サンシャインのmodeファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2017.08.25			sato		新規（ファイルのインクルード）
// *************************************************************************/


//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <hal.h>						// HAL
#include <stdio.h>
#include <search.h>
#include <parameters.h>
#include <init.h>
#include <hal_dist.h>

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


// *************************************************************************
//   機能		： 前壁（右）が閾値以上だとフラグが立つ
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 検知した:true　検知できなかった:false
// **************************    履    歴    *******************************
// 		v1.0		2017.08.28			sato		新規
// *************************************************************************/
PUBLIC BOOL MODE_DistRightCheck(void)
{
	SHORT s_rightval;
	BOOL bl_check;
	
	s_rightval = DIST_getNowVal(DIST_SEN_R_FRONT);
	
	if( s_rightval >= 100 ){
		bl_check=TRUE;
	}
	else{
		bl_check=FALSE;
	}
	
	return bl_check;
}

// *************************************************************************
//   機能		： 前壁(左)が閾値以上だとフラグが立つ
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 検知した：true	検知できなかった：false
// **************************    履    歴    *******************************
// 		v1.0		2018.8.28			sato			新規
// *************************************************************************/
PUBLIC BOOL MODE_DistLeftCheck(){
	
	SHORT 	s_leftval;
	BOOL	bl_check;
	
	s_leftval 	= DIST_getNowVal(DIST_SEN_L_FRONT);
	
	if( s_leftval >= 100 ){
		bl_check = TRUE;
	
	}else{
		bl_check = FALSE;
	
	}
	
	return bl_check;
}

// *************************************************************************
//   機能		： 手をかざすと待機状態に入る
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 両方検知：true	それ以外：false
// **************************    履    歴    *******************************
// 		v1.0		2018.8.16			吉田			新規
// *************************************************************************/
PUBLIC BOOL MODE_setWaitCheck(){
	
	BOOL bl_check;
	
	if( TRUE == MODE_DistRightCheck() ){	// 右だけ検知
		LED = LED12;

	}
	if( TRUE == MODE_DistLeftCheck() ){		// 左だけ検知
		LED = LED3;

	}
	
	if( ( TRUE == MODE_DistRightCheck() ) && ( TRUE == MODE_DistLeftCheck() ) ){
		LED = LED_ALL_ON;
		bl_check = TRUE;
		
	}else{
		bl_check = FALSE;
	
	}
	
	
	return bl_check;
}

// *************************************************************************
//   機能		： 手をかざすと待機状態に入り、かざしてから離すと実行
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 待機状態から抜け出す：true	それ以外：false
// **************************    履    歴    *******************************
// 		v1.0		2018.8.16			吉田			新規
// *************************************************************************/
PUBLIC BOOL MODE_CheckExe(){
	
	BOOL bl_check;
	
	if( TRUE == MODE_setWaitCheck() ){
		TIME_wait(200);
		
		if( FALSE == MODE_setWaitCheck() ){
			LED = LED_ALL_OFF;
			TIME_wait(1000);
			bl_check = TRUE;
			
		}else{
			bl_check = FALSE;
		
		}
		
	}else{
		
		bl_check = FALSE;
	}
	
	return bl_check;
}

// *************************************************************************
//   機能		： LED点灯用
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： 待機状態から抜け出す：true	それ以外：false
// **************************    履    歴    *******************************
// 		v1.0		2019.11.26			sato			新規
// *************************************************************************/
PUBLIC void LED_count(UCHAR number)
{
	switch(number){
		case 0:
			LED = LED_ALL_OFF;
			break;
		case 1:
			LED = LED1;
			break;
		case 2:
			LED = LED2;
			break;
		case 3:
			LED = LED3;
			break;
		case 4:
			LED = LED4;
			break;
		case 5:
			LED = LED5;
			break;
		case 6:
			LED = LED6;
			break;
		case 7:
			LED = LED7;
			break;
		case 8:
			LED = LED8;
			break;
		case 9:
			LED = LED9;
			break;
		case 10:
			LED = LED10;
			break;
		case 11:
			LED = LED11;
			break;
		case 12:
			LED = LED12;
			break;
		case 13:
			LED = LED13;
			break;
		case 14:
			LED = LED14;
			break;
		case 15:
			LED = LED_ALL_ON;;
			break;

	}
}