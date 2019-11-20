//**********************************************************************/
//  ロボット名  :half(仮称)                                           
//  概要			:メインファイル                                     
//  注意			:なし                                          
//  メモ		    :なし                                                                                        
//*********************************************************************
//   v1.0	2019.08.22		sato		新規
//*********************************************************************

//*********************************************************************
//   プロジェクト生成時に生成されたコード（使わない）
//*********************************************************************
//#include "typedefine.h"
#ifdef __cplusplus
//#include <ios>                        // Remove the comment when you use ios
//_SINT ios_base::Init::init_cnt;       // Remove the comment when you use ios
#endif

void main(void);
#ifdef __cplusplus
extern "C" {
void abort(void);
}
#endif

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>				//定義
#include <iodefine.h>				//I/O
#include <stdio.h>					//標準入出力
#include <math.h>
#include <hal.h>					//HAL
#include <init.h>
#include <parameters.h>
#include <search.h>
#include <map_cmd.h>
#include <hal_dist.h>
#include <DataFlash.h>

//**************************************************
// 定義（define）
//**************************************************


//**************************************************
// 列挙体（enum）
//**************************************************
typedef enum{
	MODE_0 = 0,					// モード0 ： バッテリー
	MODE_1,						// モード1 ： ジャイロ
	MODE_2,						// モード2 ： 距離センサ
	MODE_3,						// モード3 ： エンコーダの値をリアルタイム表示
	MODE_4,						// モード4 ： モータ回転
	MODE_5,						// モード5 ： モードの説明を記載する
	MODE_6,						// モード6 ： モードの説明を記載する
	MODE_7,						// モード7 ： 
	MODE_8,						// モード8 ： モードの説明を記載する
	MODE_9,
	MODE_10,
	MODE_11,
	MODE_12,
	MODE_13,
	MODE_14,
	MODE_15,
	MODE_MAX
}enMODE;

//**************************************************
// 構造体（struct）
//**************************************************

//**************************************************
// グローバル変数
//**************************************************
PRIVATE VUSHORT		uc_Msec;		//内部時計[msec]
PRIVATE VUSHORT		uc_Sec;			//内部時計[sec]
PRIVATE VUSHORT		uc_Min;			//内部時計[min]
PRIVATE VULONG		ul_Wait;		//1msecのwaitに使用するカウンタ[msec]
PRIVATE enMODE		en_Mode;		//現在のモード
//SPI用
PUBLIC USHORT recv;


//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************
PUBLIC void INTC_sys( void );
PRIVATE void SYS_start( void );
PUBLIC void INTC_sen( void );
PUBLIC void CTRL_pol( void );

// *************************************************************************
//   機能		： マイコンのタイマを開始する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.25			sato		新規
// *************************************************************************/
PRIVATE void RX631_staTimer(void)
{
	MTU.TSTR.BIT.CST0		= 1;//MTU0カウント開始	システム
	MTU.TSTR.BIT.CST1		= 1;//MTU1カウント開始  バッテリー監視
	MTU.TSTR.BIT.CST3		= 1;//MTU3カウント開始  センサ監視
//	TPUA.TSTR.BIT.CST1		= 1;//TPU1カウント開始  位相計測
//	TPUA.TSTR.BIT.CST2		= 1;//TPU2カウント開始  位相計測
}

// *************************************************************************
//   機能		： 時間制御の初期化。
//   注意		： 起動後、1回だけ実行する関数。
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.28			sato			新規
// *************************************************************************/
PUBLIC void TIME_init( void )
{
	/* 初期化 */
	uc_Msec = 0;		// 内部時計[msec]
	uc_Sec  = 0;		// 内部時計[sec]
	uc_Min  = 0;		// 内部時計[min]
	ul_Wait = 0;		// 1msecのwaitに使用する カウンタ[msec]
}

// *************************************************************************
//   機能		： 指定した ms 間、S/Wウェイトする。
//   注意		： なし
//   メモ		： ul_time: 待つ時間 ( msec 単位 )。 最大 600000(= 10 min) 未満
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.28			sato			新規
// *************************************************************************/
PUBLIC void TIME_wait( ULONG ul_time )
{
	ul_Wait = 0;						// 0からカウントする、"ul_Wait"は1msec毎に1加算される。

	while( ul_Wait < ul_time );			// 指定時間経過するまでウェイト
	
	return;
}


// *************************************************************************
//   機能		： 指定したフリーカウント間、S/Wウェイトする。
//   注意		： なし
//   メモ		： ul_cnt: カウント値
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.28			sato			新規
// *************************************************************************/
PUBLIC void TIME_waitFree( ULONG ul_cnt )
{
	while( ul_cnt-- );			// 0になるまでディクリメント
}

// *************************************************************************
//   機能		： モードを加算変更する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.07.02			sato			新規
// *************************************************************************/
PRIVATE void MODE_inc( void )
{
	en_Mode++;		// モードを加算
	
	/* 最大値チェック */
	if( MODE_MAX == en_Mode ){
		en_Mode = MODE_0;
	}
	
	/* モード表示 */
	switch( en_Mode ){
	
		case MODE_0:
			LED = LED_ALL_OFF;
			break;

		case MODE_1:
			LED = LED1;
			break;

		case MODE_2:
			LED = LED2;
			break;

		case MODE_3:
			LED = LED3;
			break;

		case MODE_4:
			LED = LED4;
			break;

		case MODE_5:
			LED = LED5;
			break;

		case MODE_6:
			LED = LED6;
			break;

		case MODE_7:
			LED = LED7;
			break;

		case MODE_8:
			LED = LED8;
			break;
			
		case MODE_9:
			LED = LED9;
			break;
			
		case MODE_10:
			LED = LED10;
			break;
			
		case MODE_11:
			LED = LED11;
			break;
			
		case MODE_12:
			LED = LED12;
			break;
			
		case MODE_13:
			LED = LED13;
			break;
			
		case MODE_14:
			LED = LED14;
			break;
			
		case MODE_15:
			LED = LED_ALL_ON;
			break;

		default:
			break;
	}
}

// *************************************************************************
//   機能		： モード0を実行する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2019.08.28			sato			新規
// *************************************************************************/
PRIVATE void MODE_exe0( void )
{
	/* モード表示 */
	switch( en_Mode ){
	
		case MODE_0:
			LED = LED_ALL_ON;
			while(1){
				
				printf("[電源電圧] %5.2f [mV] \n\r",BAT_getLv() );
				TIME_wait(500);
			}
			break;

		case MODE_1:
			LED = LED_ALL_ON;
			CTRL_clrData();
			GYRO_SetRef();
			while(1){
				printf("   [ジャイロ角度]%5.2f [SPIジャイロ]%x \r", 
					GYRO_getNowAngle(),recv_spi_gyro()
				);
				TIME_wait( 500 );
			}
			break;
		
		case MODE_2:
			LED = LED_ALL_ON;
			while(1){

				printf("   距離センサ [R_F]%5d [L_F]%5d [R_S]%5d [L_S]%5d \r", 
					(int)DIST_getNowVal(DIST_SEN_R_FRONT),
					(int)DIST_getNowVal(DIST_SEN_L_FRONT),
					(int)DIST_getNowVal(DIST_SEN_R_SIDE),
					(int)DIST_getNowVal(DIST_SEN_L_SIDE)
				);
				TIME_wait( 300 );
			}
			break;
		
		case MODE_3:
			LED = LED_ALL_ON;
			while(1){
				printf("[who am i]%x\n\r",recv_spi_who());
				TIME_wait(500);
			}
			break;

		case MODE_4:
			LED = LED_ALL_ON;
			while(1){
				recv_spi_encoder();
				ENC_print();
				TIME_wait(50);
			}
			break;

		case MODE_5:
			LED = LED_ALL_ON;
//			MAP_showLog();
			break;

		case MODE_6:
			LED = LED_ALL_ON;
			break;

		case MODE_7:
			LED = LED_ALL_ON;
			break;

		case MODE_8:
			LED = LED_ALL_ON;
			break;

		case MODE_9:
			LED = LED_ALL_ON;
			break;

		case MODE_10:
			LED = LED_ALL_ON;
			break;

		case MODE_11:
			LED = LED_ALL_ON;
			break;

		case MODE_12:
			LED = LED_ALL_ON;
			break;

		case MODE_13:
			LED = LED_ALL_ON;
			break;

		case MODE_14:
			LED = LED_ALL_ON;
//			turntable();
			break;

		case MODE_15://モード15は離脱用のためプログラムは設定しない
			LED = LED_ALL_ON;
			break;
			
		default:
			break;
	}
}

// *************************************************************************
//   機能		： モードを実行する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PRIVATE void MODE_exe( void )
{
//	USHORT *read;
	enMAP_HEAD_DIR		en_endDir;
	GYRO_SetRef();
	ENC_setref();
	Failsafe_flag_off();
//	log_flag_on();	//ログ関数実行用フラグ　大会時には削除
	/* モード表示 */
	switch( en_Mode ){
	
		case MODE_0:	//モジュール調整用プログラム群
			LED = LED_ALL_ON;
			en_Mode = MODE_0;	//注意：MODE_incを利用するため最初にen_Modeを初期化　最後にen_Modeを戻す操作が必要
			TIME_wait(100);
			LED = LED_ALL_OFF;
			while(1){
				if ( SW_ON == SW_INC_PIN ){
					MODE_inc();								// モードを1つ進める
					TIME_wait(SW_CHATTERING_WAIT);			// SWが離されるまで待つ
					printf("mode selecting_0\r\n");
				}
				else if (( SW_ON == SW_EXE_PIN )||(TRUE == MODE_CheckExe())){
//				else if ( SW_ON == SW_EXE_PIN ){
					MODE_exe0();								// モード実行
					TIME_wait(SW_CHATTERING_WAIT);			// SWが離されるまで待つ
					if (en_Mode == MODE_15)break;
				}

			}
			en_Mode = MODE_0;
			break;

		case MODE_1:
			LED = LED_ALL_ON;
	//		CTRL_sta();
			DCM_staMot(DCM_R);
			DCM_staMot(DCM_L);
			DCM_setDirCw(DCM_R);
			DCM_setDirCw(DCM_L);
			DCM_setPwmDuty(DCM_R,100);//*FF_BALANCE_R);
			DCM_setPwmDuty(DCM_L,100);//*FF_BALANCE_L);
			TIME_wait(1000);
			DCM_brakeMot(DCM_R);
			DCM_brakeMot(DCM_L);
		
			break;
			
		case MODE_2:
			LED = LED_ALL_ON;
			MOT_setTrgtSpeed(SEARCH_SPEED);
			MOT_setSuraStaSpeed( (FLOAT)SEARCH_SPEED );							// スラローム開始速度設定
			PARAM_setSpeedType( PARAM_ST,   PARAM_SLOW );							// [直進] 速度普通
			PARAM_setSpeedType( PARAM_TRUN, PARAM_SLOW );							// [旋回] 速度普通
			PARAM_setSpeedType( PARAM_SLA,  PARAM_SLOW );							// [スラ] 速度普通
			LED = LED_ALL_OFF;
			TIME_wait(1000);
			log_flag_on();
			MOT_goBlock_FinSpeed( 5.0, 0 );
			log_flag_off();
			break;

		case MODE_3:	
			LED = LED_ALL_ON;
			log_read2();
			break;

		case MODE_4:
			LED = LED_ALL_ON;
			MOT_setTrgtSpeed(SEARCH_SPEED);
			MOT_setSuraStaSpeed( (FLOAT)SEARCH_SPEED );							// スラローム開始速度設定
			PARAM_setSpeedType( PARAM_ST,   PARAM_SLOW );							// [直進] 速度普通
			PARAM_setSpeedType( PARAM_TRUN, PARAM_SLOW );							// [旋回] 速度普通
			PARAM_setSpeedType( PARAM_SLA,  PARAM_SLOW );							// [スラ] 速度普通
			LED = LED_ALL_OFF;
			TIME_wait(100);
			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 2500.0f, SLA_90 );		// 進入速度[mm/s]、角加速度[rad/s^2]、横G[mm/s^2]、スラロームタイプ

			MAP_setPos( 0, 0, NORTH );							// スタート位置

			log_flag_on();

			MAP_searchGoal( GOAL_MAP_X, GOAL_MAP_Y, SEARCH, SEARCH_SURA );			// ゴール設定

			log_flag_off();

//			POS_stop();			// debug
			if (( SW_ON == SW_INC_PIN )||(SYS_isOutOfCtrl() == TRUE)){}
			else{
			map_write();
			}
			/* 帰りのスラローム探索 */
			TIME_wait(1000);
			LED = LED_ALL_OFF;

//			log_flag_on();

			MAP_searchGoal( 0, 0, SEARCH, SEARCH_SURA );

//			log_flag_off();

			TIME_wait(1000);
			if (( SW_ON == SW_INC_PIN )||(SYS_isOutOfCtrl() == TRUE)){}
			else{
			map_write();
//			PARAM_setCntType( TRUE );								// 最短走行
			MAP_setPos( 0, 0, NORTH );								// スタート位置
			MAP_makeContourMap( GOAL_MAP_X, GOAL_MAP_Y, BEST_WAY );					// 等高線マップを作る
			MAP_makeCmdList( 0, 0, NORTH, GOAL_MAP_X, GOAL_MAP_Y, &en_endDir );		// ドライブコマンド作成
			MAP_makeSuraCmdList();													// スラロームコマンド作成
			MAP_makeSkewCmdList();
			}
			break;

		case MODE_5:
			LED = LED_ALL_ON;
			MOT_setTrgtSpeed(SEARCH_SPEED);
			MOT_setSuraStaSpeed( (FLOAT)SEARCH_SPEED );							// スラローム開始速度設定
			PARAM_setSpeedType( PARAM_ST,   PARAM_SLOW );							// [直進] 速度普通
			PARAM_setSpeedType( PARAM_TRUN, PARAM_SLOW );							// [旋回] 速度普通
			PARAM_setSpeedType( PARAM_SLA,  PARAM_SLOW );							// [スラ] 速度普通
			LED = LED_ALL_OFF;
			TIME_wait(100);
//			MOT_turn(MOT_R180);
			log_read2();
			break;

		case MODE_6:
			LED = LED_ALL_ON;
			MOT_setTrgtSpeed(SEARCH_SPEED);
			MOT_setSuraStaSpeed( (FLOAT)SEARCH_SPEED );							// スラローム開始速度設定
			PARAM_setSpeedType( PARAM_ST,   PARAM_SLOW );							// [直進] 速度普通
			PARAM_setSpeedType( PARAM_TRUN, PARAM_SLOW );							// [旋回] 速度普通
			PARAM_setSpeedType( PARAM_SLA,  PARAM_SLOW );							// [スラ] 速度普通

			PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 2500.0f, SLA_90 );		// 進入速度[mm/s]、角加速度[rad/s^2]、横G[mm/s^2]、スラロームタイプ

			LED = LED_ALL_OFF;
			TIME_wait(100);
			log_flag_on();
			MOT_goBlock_FinSpeed( 1.0, 300 );
			MOT_goSla(MOT_R90S,PARAM_getSra( SLA_90 ));
			MOT_goBlock_FinSpeed( 1.0, 0 );
			log_flag_off();
			break;

		case MODE_7:
			LED = LED_ALL_ON;
			MOT_setTrgtSpeed(SEARCH_SPEED);
			LED = LED_ALL_OFF;
			TIME_wait(100);
			log_flag_on();
			MOT_goBlock_FinSpeed( 7.0, 0 );
			log_flag_off();
			
			break;

		case MODE_8:
			LED = LED_ALL_ON;
			MOT_setTrgtSpeed(SEARCH_SPEED);
			LED = LED_ALL_OFF;
			TIME_wait(100);
			log_flag_on();
			MOT_turn(MOT_R90);
			log_flag_off();
			break;
			
		case MODE_9:
			LED = LED_ALL_ON;
			
			break;
			
		case MODE_10:
			LED = LED_ALL_ON;
			
			break;
			
		case MODE_11:
			LED = LED_ALL_ON;
			
			break;
			
		case MODE_12:
			LED = LED_ALL_ON;
			
			break;
			
		case MODE_13:
			LED = LED_ALL_ON;

			break;
			
		case MODE_14:
			LED = LED_ALL_ON;

			
			break;
			
		case MODE_15:
			LED = LED_ALL_ON;

			
			break;
			
		default:
			break;
	}
}

// *************************************************************************
//   機能		： main処理
//   注意		： 処理を終了させないようにする。
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.21			sato		新規
//		v1.1		2018.06.25			sato		タイマー割り込みの初期化
// *************************************************************************/
void main(void)
{	
	/* 初期化 */
	CPU_init();						// [CPU] レジスタ/GPIO/AD/TIMERなど
	TIME_init();						// [SYS] タイマー
	HAL_init();						// [HAL] ハードウエアドライバ
	DIST_init();
	MAP_Goal_init();
	RX631_staTimer();					// [CPU] タイマスタート
//	printf("set1\n\r");
	SYS_start();						// [SYS] 動作開始
	hw_dflash_init();
	MAP_init();
	
	recv_spi_init();					//SPIイニシャライズ
//	TIME_wait(100);
	GYRO_SetRef();						// [GYRO] ジャイロの基準値の取得
//	TIME_wait(20);

	/* big roop */
	while(1){
		if ( SW_ON == SW_INC_PIN ){
			MODE_inc();								// モードを1つ進める
			TIME_wait(SW_CHATTERING_WAIT);			// SWが離されるまで待つ
		printf("mode selecting\r\n");
		}
		else if (( SW_ON == SW_EXE_PIN )||(TRUE == MODE_CheckExe())){
//		else if ( SW_ON == SW_EXE_PIN ){
			MODE_exe();								// モード実行
			TIME_wait(SW_CHATTERING_WAIT);			// SWが離されるまで待つ
		}

	}
}


// *************************************************************************
//   機能		： 割り込み関数、システム用(msecタイマ)
//   注意		： なし
//   メモ		： MTU0のTGRA割り込み、1msec毎に関数が実行される。
//				   正常にマイコンが動作している場合、1msec周期でLEDが点滅する
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.23			sato		新規
// *************************************************************************/
PUBLIC void INTC_sys(void)
{
//	static USHORT i = 0;
	
	/*システム動作確認LED*/
/*	if( i==500 ){
//		LED0 = ~LED0;	//Lチカ
		i=0;
	}
	else{
		i++;
	}
*/	
	/* ---------- */
	/*  内部時計  */
	/* ---------- */
	uc_Msec++;					// msec
	if( uc_Msec > 999 ){		// msec → sec
		uc_Msec  = 0;
		uc_Sec++;
	}
	if( uc_Sec > 59 ){			// sec → min
		uc_Sec = 0;
		uc_Min++;
	}
	
	/* ----------------------- */
	/*  S/Wウェイト・カウンタ  */
	/* ----------------------- */
	ul_Wait++;
	ul_Wait %= 6000000;			// 10 min (= 6000000 カウント) で 0 クリア
	
	CTRL_pol();
}

// *************************************************************************
//   機能		： 動作を開始する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.25			外川			新規
// *************************************************************************/
PRIVATE void SYS_start( void )
{
//	UCHAR	i;

	/* LED上位⇔下位4bit点滅 */
/*	for( i = 0; i < 2; i++ ){
	
		LED4 = 0x03;
		TIME_wait(50);
		LED4 = 0x0C;
		TIME_wait(50);
	}*/
/*	LEDR = ON;
	TIME_wait(100);
	LEDR = OFF;
	LEDG = ON;
	TIME_wait(100);
	LEDG = OFF;
*/
	/* タイトル表示 */
	printf(" ┏━━━━━━━━━━━━━━━━━━━━━┓\r\n");
	printf(" ┃ Robo Name  : サンシャイン(・ω・)/~~     ┃\r\n");
	printf(" ┃ Developer  : T.Togawa & Oki              ┃\r\n");
	printf(" ┃ Version    : Protype1号機                ┃\r\n");
	printf(" ┃ Target     : 教育用                      ┃\r\n");
	printf(" ┃ Project By : Hosei Univ. Denken Group    ┃\r\n");
	printf(" ┗━━━━━━━━━━━━━━━━━━━━━┛\r\n");

	PARAM_makeSra( (FLOAT)SEARCH_SPEED, 100.0f, 2500.0f, SLA_45 );		// 進入速度[mm/s]、角加速度[rad/s^2]、横G[mm/s^2]、スラロームタイプ200 2000	T	200 2000
	PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 2500.0f, SLA_90 );		// 進入速度[mm/s]、角加速度[rad/s^2]、横G[mm/s^2]、スラロームタイプ
	PARAM_makeSra( (FLOAT)SEARCH_SPEED, 150.0f, 6000.0f, SLA_135 );		// 進入速度[mm/s]、角加速度[rad/s^2]、横G[mm/s^2]、スラロームタイプ300 4500		300 4000
	PARAM_makeSra( (FLOAT)SEARCH_SPEED, 200.0f, 7000.0f, SLA_N90 );		// 進入速度[mm/s]、角加速度[rad/s^2]、横G[mm/s^2]、スラロームタイプ500 5000		500 5000

//	printf("600\n\r");
//	PARAM_makeSra( (FLOAT)600, 350.0f, 4500.0f, SLA_45 );		// 進入速度[mm/s]、角加速度[rad/s^2]、横G[mm/s^2]、スラロームタイプ200 2000	T	200 2000
//	PARAM_makeSra( (FLOAT)600, 350.0f, 5000.0f, SLA_90 );		// 進入速度[mm/s]、角加速度[rad/s^2]、横G[mm/s^2]、スラロームタイプ300 3500		200 4000
//	PARAM_makeSra( (FLOAT)600, 400.0f, 6500.0f, SLA_135 );		// 進入速度[mm/s]、角加速度[rad/s^2]、横G[mm/s^2]、スラロームタイプ300 4500		300 4000
//	PARAM_makeSra( (FLOAT)600, 550.0f, 7500.0f, SLA_N90 );		// 進入速度[mm/s]、角加速度[rad/s^2]、横G[mm/s^2]、スラロームタイプ500 5000		500 5000

	
}

// *************************************************************************
//   機能		： 割り込み関数、センサ用
//   注意		： なし
//   メモ		： MTU4のTGRA割り込み、0.25msec毎に関数が実行される。
//				： 正常にマイコンが動作している場合、250usec周期で
//				： 距離センサ0 → 距離センサ1 → 距離センサ2 → ジャイロセンサ の順でスキャンする。
//				： 従って、1センサのスキャン周期は1msecとなる。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.04			外川			新規
// *************************************************************************/
PUBLIC void INTC_sen( void )
{
	static UCHAR	i = 0;
	
	/* センサ処理  */
	switch( i ){
		case 0:		// ジャイロセンサ
			GYRO_Pol();
			break;
		
		case 1:		// 前壁センサ
			DIST_Pol_Front();
			break;
		
		case 2:		// 横壁センサ
			DIST_Pol_Side();
			break;
		
		case 3:		// 斜め壁センサ
			// 斜めセンサは使用しない
			break;
		
	}
	
	i = ( i + 1 ) % 4;			// roop
	
	return;
}

//使わないコード（初期）
#ifdef __cplusplus
void abort(void)
{

}
#endif
