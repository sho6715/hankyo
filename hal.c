// *************************************************************************
//   ロボット名	： 電研ベーシックDCマウス、サンシャイン
//   概要		： サンシャインのHAL（ハードウエア抽象層）ファイル
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.06.28			sato			新規（ファイルのインクルード）
// *************************************************************************/

//**************************************************
// インクルードファイル（include）
//**************************************************
#include <typedefine.h>						// 定義
#include <iodefine.h>						// I/O
#include <hal.h>							// HAL
#include <stdio.h>
#include <math.h>
#include <init.h>
#include <parameters.h>
//#include <hal_dist.h>

//**************************************************
// 定義（define）
//**************************************************
#define		BAT_GOOD			(3060)			// 残量減ってきた（黄色）、1セル3.7V以上： 3060 = ( 3700mV * 1セル ) / 1.5(分圧) / 3300 * 4096 - 1
#define		BAT_LOW				(2812)			// 残量やばい！！（赤色）、1セル3.4V以上： 2812 = ( 3400mV * 1セル ) / 1.5(分圧) / 3300 * 4096 - 1
#define		GYRO_REF_NUM		(200)		//ジャイロのリファレンス値をサンプリングする数
#define		ENC_RESET_VAL		(32768)			// エンコーダの中間値
#define		DCM_R_IN			(PORT3.PODR.BIT.B1)		// DCM右IN
#define		DCM_L_IN			(PORT1.PODR.BIT.B5)		// DCM左IN
#define		DCM_R_TIMER			(TPUA.TSTR.BIT.CST0)	// DCM右タイマ開始
#define		DCM_L_TIMER			(TPUA.TSTR.BIT.CST1)	// DCM左タイマ開始
#define		DCM_R_TIORB			(TPU0.TIORH.BIT.IOB)	// DCM右ピン出力設定B
#define		DCM_L_TIORB			(TPU1.TIOR.BIT.IOB)	// DCM左ピン出力設定B
#define		DCM_R_TCNT			(TPU0.TCNT)				// DCM右カウント値
#define		DCM_L_TCNT			(TPU1.TCNT)				// DCM左カウント値
#define		DCM_R_GRA			(TPU0.TGRA)				// DCM右周期
#define		DCM_R_GRB			(TPU0.TGRB)				// DCM右Duty比
#define		DCM_L_GRA			(TPU1.TGRA)				// DCM左周期
#define		DCM_L_GRB			(TPU1.TGRB)				// DCM左Duty比

/* 非調整パラメータ */
#define PI							( 3.14159f )								// π

/* 調整パラメータ */
#define VCC_MAX						( 8.4f )									// バッテリ最大電圧[V]、4.2[V]×2[セル]
#define TIRE_R						( 22.25f )									// タイヤ直径 [mm]
#define GEAR_RATIO					( 36 / 8 )									// ギア比(スパー/ピニオン)
//#define ROTATE_PULSE					( 2048 )									// 1周のタイヤパルス数
#define DIST_1STEP					( PI * TIRE_R / GEAR_RATIO / ROTATE_PULSE )				// 1パルスで進む距離 [mm]
#define F_CNT2MM(cnt)					( (FLOAT)cnt * DIST_1STEP )				// [カウント値]から[mm]へ換算
#define MOT_MOVE_ST_THRESHOLD				( 25 )							// 直進移動距離の閾値[mm]
#define MOT_MOVE_ST_MIN					( 20 )							// 直進移動距離の最低移動量[mm]
//#define MOT_ACC						( 1800 )						// 直進移動の加速度[mm/s2]
//#define MOT_DEC						( 1800 )						// 直進移動の減速度[mm/s2]
//#define MOT_ACC_ANGLE					( 1800 )		//旋回の角加速度[mm/s2]
//#define MOT_DEC_ANGLE					( 1800 )		//旋回の角減速度[mm/s2]

//20170815 超信地旋回実装時に追加
#define A1_MIN					( 25 )						// 第1最低移動角度
#define A2_MIN					( 30 )						// 第2最低移動角度
#define A3_MIN					( 20 )						// 第3最低移動角度

#define ANGLE_OFFSET1_R				( 0 )	//-12					// 角度のオフセット値（バッファリングによる誤差を埋めるための値）
#define ANGLE_OFFSET1				( 0 )	//-12					// 角度のオフセット値（バッファリングによる誤差を埋めるための値）
#define ANGLE_OFFSET2_R				( 0 )	//3
#define ANGLE_OFFSET2				( 0 )						// 角度のオフセット値（バッファリングによる誤差を埋めるための値）
#define ANGLE_OFFSET3				( 0 )					// 角度のオフセット値（バッファリングによる誤差を埋めるための値）

//#define MOT_MOVE_ST_THRESHOLD			( 25 )						// 直進移動距離の閾値[mm]
//#define MOT_MOVE_ST_MIN				( 20 )						// 直進移動距離の最低移動量[mm]


#define log_num			(1000)					//ログ取得数（変更時はこちらを変更）

//**************************************************
// 列挙体（enum）
//**************************************************
/* 制御動作タイプ */
typedef enum{
	CTRL_ACC,				// [00] 加速中(直進)
	CTRL_CONST,				// [01] 等速中(直進)
	CTRL_DEC,				// [02] 減速中(直進)

	CTRL_SKEW_ACC,			// [03] 斜め加速中(直進)
	CTRL_SKEW_CONST,		// [04] 斜め等速中(直進)
	CTRL_SKEW_DEC,			// [05] 斜め減速中(直進)
	
	CTRL_HIT_WALL,			// [06]壁当て動作
	
	CTRL_ACC_TRUN,			// [07] 加速中(超信地旋回)
	CTRL_CONST_TRUN,		// [08] 等速中(超信地旋回)
	CTRL_DEC_TRUN,			// [09] 減速中(超信地旋回)
	
	CTRL_ENTRY_SURA,		// [10]スラローム前前進
	CTRL_ACC_SURA,			// [11] 加速中(スラ)
	CTRL_CONST_SURA,		// [12] 等速中(スラ)
	CTRL_DEC_SURA,			// [13] 減速中(スラ)
	CTRL_EXIT_SURA,			// [14] スラローム後前進

	CTRL_MAX,

}enCTRL_TYPE;


/* 動作タイプ */
typedef enum{
	MOT_ST_NC    =  0,
	MOT_ACC_CONST_DEC,			// [01] 台形加速
	MOT_ACC_CONST_DEC_CUSTOM,	// [02] 台形加速（等速値変更）
	MOT_ACC_CONST,				// [03] 加速＋等速
	MOT_ACC_CONST_CUSTOM,		// [04] 加速＋等速（加速値変更）
	MOT_CONST_DEC,				// [05] 等速＋減速
	MOT_CONST_DEC_CUSTOM,		// [06] 等速＋減速（減速値変更）
	MOT_ST_MAX,
}enMOT_ST_TYPE;

/* 直進タイプ */
typedef enum{
	MOT_GO_ST_NORMAL    =  0,	// 通常の直進
	MOT_GO_ST_SKEW,				// 斜めの直進
	MOT_GO_ST_MAX,
}enMOT_GO_ST_TYPE;


//**************************************************
// 構造体（struct）
//**************************************************
/* 動作情報 */
typedef struct{

	FLOAT			f_time;			// 時間					[msec]

	/* 速度制御 */
	FLOAT			f_acc1;			// 加速度1				[mm/s2]
	FLOAT			f_acc3;			// 加速度3				[mm/s2]
	FLOAT			f_now;			// 現在速度				[mm/s]
	FLOAT			f_trgt;			// 加速後の目標速度		[mm/s]
	FLOAT			f_last;			// 減速後の最終速度		[mm/s]

	/* 距離制御 */
	FLOAT			f_dist;			// 移動距離				[mm]
	FLOAT			f_l1;			// 第1移動距離			[mm]
	FLOAT			f_l1_2;			// 第1+2移動距離		[mm]

	/* 角速度制御 */
	FLOAT			f_accAngleS1;	// 角加速度1			[rad/s2]
	FLOAT			f_accAngleS3;	// 角加速度3			[rad/s2]
	FLOAT			f_nowAngleS;	// 現在角速度			[rad/s]
	FLOAT			f_trgtAngleS;	// 加速後の目標角速度	[rad/s]
	FLOAT			f_lastAngleS;	// 減速後の最終角速度	[rad/s]

	/* 角度制御 */
	FLOAT			f_angle;		// 移動角度				[rad]
	FLOAT			f_angle1;		// 第1移動角度			[rad]
	FLOAT			f_angle1_2;		// 第1+2移動角度		[rad]
}stMOT_DATA;

/* 制御データ */
typedef struct{
	enCTRL_TYPE		en_type;		// 動作タイプ
	FLOAT			f_time;			// 目標時間 [sec]
	FLOAT			f_acc;			// [速度制御]   加速度[mm/s2]
	FLOAT			f_now;			// [速度制御]   現在速度[mm/s]
	FLOAT			f_trgt;			// [速度制御]   最終速度[mm/s]
	FLOAT			f_nowDist;		// [距離制御]   現在距離[mm]
	FLOAT			f_dist;			// [距離制御]   最終距離[mm]
	FLOAT			f_accAngleS;	// [角速度制御] 角加速度[rad/s2]
	FLOAT			f_nowAngleS;	// [角速度制御] 現在角速度[rad/s]
	FLOAT			f_trgtAngleS;	// [角速度制御] 最終角速度[rad/s]
	FLOAT			f_nowAngle;		// [角度制御]   現在角度[rad]
	FLOAT			f_angle;		// [角速制御]   最終角度[rad]
}stCTRL_DATA;


//**************************************************
// グローバル変数
//**************************************************
/* バッテリ監視 */
PRIVATE USHORT	us_BatLvAve = 4095;							// バッテリ平均値（AD変換の最大値で初期化）

/*ジャイロセンサ*/
PRIVATE SHORT s_GyroVal; 					  				// ジャイロセンサの現在値
PRIVATE SHORT s_GyroValBuf[8];								// ジャイロセンサのバッファ値
PUBLIC FLOAT  f_GyroNowAngle;		 						// ジャイロセンサの現在角度
PRIVATE LONG  l_GyroRef; 									// ジャイロセンサの基準値

/* 制御  */
PRIVATE enCTRL_TYPE		en_Type;						// 制御方式
PRIVATE UCHAR 			uc_CtrlFlag			= FALSE;	// フィードバック or フィードフォワード 制御有効フラグ（FALSE:無効、1：有効）
PRIVATE LONG			l_CntR;							// 右モータのカウント変化量						（1[msec]毎に更新される）
PRIVATE LONG			l_CntL;							// 左モータのカウント変化量						（1[msec]毎に更新される）
// 制御
PUBLIC  FLOAT			f_Time 				= 0;		// 動作時間[sec]								（1[msec]毎に更新される）
PUBLIC  FLOAT			f_TrgtTime 			= 1000;		// 動作目標時間 [msec]							（設定値）
// 速度制御//////////////////////////////////////////
PRIVATE FLOAT 			f_Acc			= 0;		// [速度制御]   加速度							（設定値）
PRIVATE FLOAT			f_BaseSpeed		= 0;		// [速度制御]   初速度							（設定値）
PRIVATE FLOAT			f_LastSpeed 		= 0;		// [速度制御]   最終目標速度					（設定値）
PRIVATE FLOAT			f_NowSpeed		= 0;		// [速度制御]   現在の速度 [mm/s]				（1[msec]毎に更新される）
PUBLIC FLOAT			f_TrgtSpeed 		= 0;		// [速度制御]   目標移動速度 [mm/s]				（1[msec]毎に更新される）
PRIVATE FLOAT			f_ErrSpeedBuf		= 0;		// [速度制御] 　速度エラー値のバッファ	（1[msec]毎に更新される）
PUBLIC FLOAT			f_SpeedErrSum 		= 0;		// [速度制御]   速度積分制御のサム値			（1[msec]毎に更新される）
// 距離制御
PRIVATE FLOAT			f_BaseDist		= 0;		// [距離制御]   初期位置						（設定値）
PRIVATE FLOAT			f_LastDist 		= 0;		// [距離制御]   最終移動距離					（設定値）
PUBLIC FLOAT			f_TrgtDist 		= 0;		// [距離制御]   目標移動距離					（1[msec]毎に更新される）
PUBLIC volatile FLOAT 		f_NowDist		= 0;		// [距離制御]   現在距離						（1[msec]毎に更新される）
PRIVATE FLOAT			f_NowDistR		= 0;		// [距離制御]   現在距離（右）					（1[msec]毎に更新される）
PRIVATE FLOAT 			f_NowDistL		= 0;		// [距離制御]   現在距離（左）					（1[msec]毎に更新される）
PUBLIC FLOAT			f_DistErrSum 		= 0;		// [距離制御]   距離積分制御のサム値			（1[msec]毎に更新される）
// 角速度制御
PRIVATE FLOAT 			f_AccAngleS		= 0;		// [角速度制御] 角加速度						（設定値）
PRIVATE FLOAT			f_BaseAngleS		= 0;		// [角速度制御] 初期角速度						（設定値）
PRIVATE FLOAT			f_LastAngleS 		= 0;		// [角速度制御] 最終目標角速度					（設定値）
PUBLIC FLOAT			f_TrgtAngleS 		= 0;		// [角速度制御] 目標角速度 [rad/s]				（1[msec]毎に更新される）
PRIVATE FLOAT			f_ErrAngleSBuf		= 0;		// [角速度制御] 角速度エラー値のバッファ	（1[msec]毎に更新される）
PUBLIC FLOAT			f_AngleSErrSum 		= 0;		// [角速度制御]   角度積分制御のサム値			（1[msec]毎に更新される）
// 角度制御
PRIVATE FLOAT			f_BaseAngle		= 0;		// [角度制御]   初期角度						（設定値）
PRIVATE FLOAT			f_LastAngle 		= 0;		// [角度制御]   最終目標角度					（設定値）
PUBLIC volatile FLOAT 		f_NowAngle		= 0;		// [角度制御]   現在角度　	volatileをつけないとwhileから抜けられなくなる（最適化のせい）（1[msec]毎に更新される）
PUBLIC FLOAT			f_TrgtAngle 		= 0;		// [角度制御]   目標角度						（1[msec]毎に更新される）
PUBLIC FLOAT			f_AngleErrSum 		= 0;		// [角度制御]   角度積分制御のサム値			（1[msec]毎に更新される）
// 壁制御
PRIVATE LONG 			l_WallErr 		= 0;		// [壁制御]     壁との偏差						（1[msec]毎に更新される）
PRIVATE FLOAT			f_ErrDistBuf		= 0;		// [壁制御]     距離センサーエラー値のバッファ	（1[msec]毎に更新される）

/* 動作 */
PRIVATE FLOAT 			f_MotNowSpeed 		= 0.0f;		// 現在速度
PRIVATE FLOAT 			f_MotTrgtSpeed 		= 0.0f;		// 目標速度
PRIVATE	stMOT_DATA 		st_Info;				// シーケンスデータ
PRIVATE FLOAT			f_MotSuraStaSpeed	= 0.0f;
PRIVATE FLOAT			sliplengs		= 0.0f;		//スリップ距離

PRIVATE enMOT_WALL_EDGE_TYPE	en_WallEdge = MOT_WALL_EDGE_NONE;	// 壁切れ補正
PRIVATE BOOL			bl_IsWallEdge = FALSE;				// 壁切れ検知（TRUE:検知、FALSE：非検知）
PRIVATE FLOAT			f_WallEdgeAddDist = 0;				// 壁切れ補正の移動距離

//フェイルセーフ
PUBLIC FLOAT  f_ErrChkAngle; 			  // ジャイロセンサのエラー検出用の角度
PUBLIC BOOL   bl_ErrChk; 				  // ジャイロセンサのエラー検出（FALSE：検知しない、TRUE：検知する）
PRIVATE BOOL			bl_failsafe		= FALSE;	// マウスがの制御不能（TRUE：制御不能、FALSE：制御可能）

//ログプログラム群（取得数変更はdefineへ）
PRIVATE	FLOAT	Log_1[log_num];
PRIVATE FLOAT	Log_2[log_num];
PRIVATE FLOAT	Log_3[log_num];
PRIVATE FLOAT	Log_4[log_num];
PRIVATE FLOAT	Log_5[log_num];
PRIVATE FLOAT	Log_6[log_num];
PRIVATE FLOAT	Log_7[log_num];
PRIVATE FLOAT	Log_8[log_num];
PRIVATE FLOAT	Log_9[log_num];
PRIVATE FLOAT	Log_10[log_num];
PRIVATE FLOAT	Log_11[log_num];
PRIVATE FLOAT	Log_12[log_num];

PRIVATE	USHORT	log_count = 0;
PUBLIC	BOOL	b_logflag = FALSE;

PRIVATE FLOAT	templog1	= 0;
PRIVATE FLOAT	templog2	= 0;

//ログ用デューティー
PRIVATE	FLOAT	f_Duty_R;
PRIVATE	FLOAT	f_Duty_L;

PRIVATE CHAR	i;

//**************************************************
// プロトタイプ宣言（ファイル内で必要なものだけ記述）
//**************************************************


// *************************************************************************
//   機能		： HALを初期化する。
//   注意		： なし
//   メモ		： 内部変数などをクリアする。
//   引数		： なし
//   返り値		： なし
//   その他     ：起動時に動作
// **************************    履    歴    *******************************
// 		v1.0		2013.11.27			外川			新規
// 		v1.1		2013.11.27			外川			センサの基準値とリミット値を設定
// *************************************************************************/
PUBLIC void HAL_init( void )
{
	/* ジャイロセンサ */
	f_GyroNowAngle = 0;			// ジャイロセンサの現在角度(0にしても探索他は動くが、宴会とかtestrunとかは動かない)修正済みと思われる
	l_GyroRef  = 0;				// ジャイロセンサの基準値
	
	f_ErrChkAngle = 0;
	bl_ErrChk = FALSE;
	
	/* エンコーダ */
//	ENC_Sta();	//左側のピンがLEFT(突起は前方方向)
	
	/* DCM*/
//	DCM_ENA = ON;
	
	/* [暫定] warningを消すだけ（削除してOK） */
	f_AccAngleS = f_AccAngleS;
	f_BaseAngleS = f_BaseAngleS;
	f_LastAngleS = f_LastAngleS;
	f_BaseAngle = f_BaseAngle;
	f_LastAngle = f_LastAngle;

}

// *************************************************************************
//   機能		： 1文字出力
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC void SCI_putch(char data)
{
	while(SCI1.SSR.BIT.TEND == 0) {};
//		if (SCI1.SSR.BYTE & 0x80) {			// 送信バッファの空きチェック
			SCI1.TDR = data;
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x40;
			SCI1.SSR.BIT.TEND = 0;
//			break;
//		}
//	}
}


// *************************************************************************
//   機能		： 文字列出力
//   注意		： なし
//   メモ		： "\n"のみで"CR+LF"出力を行う。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC void SCI_puts(char *buffer)
{
	char data;
	
	/* nullまで出力 */
	while( (data = *( buffer++ ) ) != 0 ){
		
		/* データの値に応じて出力を変える */
		if (data == 0x0a) {
			SCI_putch(0x0d);		// CR出力
			SCI_putch(0x0a);		// LF出力
		} else {
			SCI_putch(data);		// 1文字出力
		}
	}
}


// *************************************************************************
//   機能		： 文字列出力
//   注意		： なし
//   メモ		： 文字列長さ指定付き"/\n"のみ "CR+LF"出力を行う。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC void SCI_putsL(char *buffer, int len)
{
	int i;
	char data;
	
	for( i=0; i<len; i++ ){
		data=*(buffer++);
		
		/* データの値に応じて出力を変える */
		if (data == 0x0a) {
			SCI_putch(0x0d);		// CR出力
			SCI_putch(0x0a);		// LF出力
		} else {
			SCI_putch(data);		// 1文字出力
		}
	}
}


// *************************************************************************
//   機能		： 1文字出力用ラッパー関数
//   注意		： なし
//   メモ		： printfなどの低レベル出力に使用される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC void charput(unsigned char data)
{
	SCI_putch(data);
}


// *************************************************************************
//   機能		： 入力バッファチェック
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC int SCI_chkRecv(void)
{
	/* データの受信チェック */
	if (IR(SCI1,RXI1) == 1) {
		return 1;		// 受信データあり
	}
	else {
		return 0;		// 受信データなし
	}
}


// *************************************************************************
//   機能		： 1文字入力（char型版）
//   注意		： なし
//   メモ		： エコーバックなし。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC char SCI_getch(void)
{
	char data;
	
	while(1){
		/* データの受信チェック */
		if ( SCI_chkRecv() ) {
			data = SCI1.RDR;						// データ受信
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x80;
			IR(SCI1,RXI1) = 0;
			break;
		}
	}
	return data;
}


// *************************************************************************
//   機能		： 1文字入力（unsigned char型版）
//   注意		： なし
//   メモ		： エコーバックなし。
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC unsigned char SCI_getch_uc(void)
{
	unsigned char data;
	while(1){
		/* データの受信チェック */
		if ( SCI_chkRecv() ) {
			data = SCI1.RDR;						// データ受信
//			SCI1.SSR.BYTE = SCI1.SSR.BYTE & 0x80;
			IR(SCI1,RXI1) = 0;
			break;
		}
	}
	return data;
}


// *************************************************************************
//   機能		： 文字列入力
//   注意		： なし
//   メモ		： CRコードまで/最大255文字/エコーバックあり
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC int SCI_gets(char *buffer)
{
	char data;
	int  i = 0;

	while(1){
		data = SCI_getch();		// 1文字入力
		*buffer = data;
		SCI_putch(data);		// 1文字出力(エコーバック)
		buffer++;
		i++;
		if (i > 255)      break;	// 最大文字数に到達
		if (data == 0x0D) break;	// CRコードまで受信した
	}
	*buffer = (unsigned char)0;		// null
	
	return i;						// 入力文字数を返却
}


// *************************************************************************
//   機能		： 1文字入力用ラッパー関数
//   注意		： なし
//   メモ		： scanfなどの低レベル出力に使用される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC unsigned char charget(void)
{
	return SCI_getch_uc();
}


// *************************************************************************
//   機能		： バッテリ電圧を取得する
//   注意		： なし
//   メモ		： 直前5回の平均値
//   引数		： なし
//   返り値		： 電圧[mV]
// **************************    履    歴    *******************************
// 		v1.0		2013.11.13			外川			新規
// *************************************************************************/
PUBLIC FLOAT BAT_getLv(void)
{
	FLOAT f_val = (FLOAT)( us_BatLvAve + 1 );		// 値は0から始まるから1を加算
	
	return ( f_val / 4096 * 3300 / 2 * 3 );
}


// *************************************************************************
//   機能		： バッテリ監視用ポーリング関数
//   注意		： なし
//   メモ		： 割り込みから実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.16			外川			新規
// *************************************************************************/
PUBLIC void BAT_Pol( void )
{
	static USHORT 	us_batLv[5] = { 4095, 4095, 4095, 4095, 4095 };		// バッテリレベル（AD変換の最大値で初期化）

	/* ================================================== */
	/*  平均値を取得するため、データのバッファ処理を行う  */
	/* ================================================== */
	/* バッファをシフト */
	us_batLv[4] = us_batLv[3];
	us_batLv[3] = us_batLv[2];
	us_batLv[2] = us_batLv[1];
	us_batLv[1] = us_batLv[0];

	/* 最新の値を格納 */
	S12AD.ADANS0.WORD 		= 0x0001;		// AN0 変換対象設定
	S12AD.ADCSR.BIT.ADST 		= 1;		// AD変換開始
	while( S12AD.ADCSR.BIT.ADST == 1);		// AD変換待ち
	us_batLv[0] = S12AD.ADDR0;				// AN0 変換データ取得

	/* 電圧平均化 */
	us_BatLvAve = ( us_batLv[0] + us_batLv[1] + us_batLv[2] + us_batLv[3] + us_batLv[4] ) / 5;
	
	/*  残量に応じてLEDを表示  */
	/* ======================= */
	if( us_BatLvAve < BAT_LOW ) {			// 残量やばい！！（赤色）
		LEDG = OFF;
//		LEDR = ON;
	}
	else if( us_BatLvAve < BAT_GOOD ) {		// 残量減ってきた（黄色）
//		LEDG = ON;
//		LEDR = ON;
		
		if( i==50 ){
			LEDG = ~LEDG;	//Lチカ
			i=0;
		}
		else{
			i++;
		}
		
	}
	else{									// 残量問題なし（緑色）
		LEDG = ON;
//		LEDR = OFF;
	}
}

// *************************************************************************
//   機能		： ジャイロセンサのリファレンス値（基準の値）を設定する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//   その他　　：　起動時に動作
// **************************    履    歴    *******************************
// 		v1.0		2018.08.07			sato			新規
//		v1.1		2018.08.25			sato			Refの取得が正確にできていないように見えたため強引にRefを設定している（緊急措置）
// *************************************************************************/
PUBLIC void GYRO_SetRef( void )
{
	USHORT i;
	ULONG ul_ref = 0;
	
	/* データサンプリング */
	for( i=0; i<GYRO_REF_NUM; i++){			// 100回サンプリングした平均値を基準の値とする。
		ul_ref += (ULONG)s_GyroVal;
		TIME_wait(1);
	}
	
	/* 基準値算出（平均値） */
	l_GyroRef = ul_ref / GYRO_REF_NUM * 100;		// 精度を100倍にする
//	l_GyroRef = 0x1304*100;
}

// *************************************************************************
//   機能		： ジャイロの角速度に関する制御偏差を取得する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
//
// **************************    履    歴    *******************************
// 		v1.0		2013.11.26		外川			新規
// *************************************************************************/
PUBLIC FLOAT GYRO_getSpeedErr( void )
{
	LONG  l_val = (LONG)s_GyroVal * 100 ;				// 100倍の精度にする
	LONG  l_err = l_val - l_GyroRef ;
	FLOAT f_res;
	
	/* 角速度の偏差算出 */
	if( ( l_err < -20 * 100 ) || ( 20 * 100 < l_err ) ){
		f_res = (FLOAT)l_err /32.768 / 100;		//32.768 = 2^16(16bit)/2000(+-1000度) LSB/(°/s)
													// 100倍の精度
	}
	else{
		f_res = 0;									// [deg/s]
	}
	
	return f_res;
}

// *************************************************************************
//   機能		： ジャイロの現在の角度を取得する
//   注意		： なし
//   メモ		： ☆
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.26			外川			新規
// *************************************************************************/
PUBLIC FLOAT GYRO_getNowAngle( void )
{
	return f_GyroNowAngle;
}

// *************************************************************************
//   機能		： ジャイロの現在の角度を取得する
//   注意		： なし
//   メモ		： ☆
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.26			外川			新規
// *************************************************************************/
PUBLIC FLOAT GYRO_getRef( void )
{
	return l_GyroRef;
}

// *************************************************************************
//   機能		： ジャイロセンサ用ポーリング関数
//   注意		： なし
//   メモ		： 割り込みから実行される
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.11.26			外川			新規
//		v2.0		2018.08.16			sato		SPIによるジャイロ取得設定
// *************************************************************************/
PUBLIC void GYRO_Pol( void )
{
	FLOAT f_speed;
	
	/* バッファシフト（[7]に新しいデータを入れるため、[0]のデータを捨てて、1つずつ詰める） */
/*	s_GyroValBuf[0]	= s_GyroValBuf[1];
	s_GyroValBuf[1]	= s_GyroValBuf[2];
	s_GyroValBuf[2]	= s_GyroValBuf[3];
	s_GyroValBuf[3]	= s_GyroValBuf[4];
	s_GyroValBuf[4]	= s_GyroValBuf[5];
	s_GyroValBuf[5]	= s_GyroValBuf[6];
	s_GyroValBuf[6]	= s_GyroValBuf[7];
*/	
	/* 最新のジャイロセンサ値を取得 */
//	s_GyroValBuf[7] = (SHORT)recv_spi_gyro();
	
	/* ジャイロの値を平滑する（平滑数は8つ） */
	s_GyroVal = (SHORT)recv_spi_gyro();//( s_GyroValBuf[0] + s_GyroValBuf[1] + s_GyroValBuf[2] + s_GyroValBuf[3] +
				//  s_GyroValBuf[4] + s_GyroValBuf[5] + s_GyroValBuf[6] + s_GyroValBuf[7] ) / 8;
	
	/* 現在の角度を更新する */
	f_speed = GYRO_getSpeedErr();			// 角速度取得 (0.001sec毎の角速度)
	f_GyroNowAngle += f_speed / 1000;		// 角度設定   (0.001sec毎に加算するため)

	/* エラーチェック */
	if( bl_ErrChk == TRUE ){
		
		f_ErrChkAngle += f_speed/1000;		// 角度設定   (0.001sec毎に加算するため)
		
		if( ( f_ErrChkAngle < -500 ) || ( 500 < f_ErrChkAngle )||(f_speed <-1000)||(1000<f_speed) ){
			
//			Failsafe_flag();
//			printf("fail\n\r");
		}
	}
}

// *************************************************************************
//   機能		： エラー検出用を開始する
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.10			sato			新規
// *************************************************************************/
PUBLIC void GYRO_staErrChkAngle( void )
{
	f_ErrChkAngle = 0;
	bl_ErrChk = TRUE;

}


// *************************************************************************
//   機能		： エラー検出用を終了する
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.10.10			sato			新規
// *************************************************************************/
PUBLIC void GYRO_endErrChkAngle( void )
{
	f_ErrChkAngle = 0;
	bl_ErrChk = FALSE;

}

// *************************************************************************
//   機能		： SPI関数
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.06			sato			新規
// *************************************************************************/
PUBLIC USHORT recv_spi(USHORT spi_ad)
{
	USHORT recv;
	RSPI0.SPDR.WORD.H = spi_ad;
	
	while(!RSPI0.SPSR.BIT.IDLNF);	//送信開始を確認
	while(RSPI0.SPSR.BIT.IDLNF);		//RSPI0ｱｲﾄﾞﾙ状態か確認
		recv = RSPI0.SPDR.WORD.H ;

	return(recv);
}

// *************************************************************************
//   機能		： SPI(whoami)
//   注意		： なし
//   メモ		： 
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.04			外川			新規
// *************************************************************************/
PUBLIC USHORT recv_spi_who(void)
{
	USHORT recv;
	USHORT whoami = (0x75|0x80);
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(whoami);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	return(recv);
		
}

// *************************************************************************
//   機能		： SPI_init
//   注意		： なし
//   メモ		： 初回実行
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.05			sato		新規
// *************************************************************************/
PUBLIC void recv_spi_init(void)
{
	USHORT recv;
	USHORT register107 = (0x6B|0x00);		//power management1
	USHORT register106 = (0x6A|0x00);
	USHORT register112 = (0x70|0x00);
	USHORT register27 = (0x1B|0x00);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register107);
	recv = recv_spi(0x80);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(100);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register106);
	recv = recv_spi(0x01);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(100);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register112);
	recv = recv_spi(0x40);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register27);
	recv = recv_spi(0x30);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register107);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	TIME_wait(1);
	
	/*read*/
/*
	PORTC.PODR.BIT.B4 = 0;
	recv = recv_spi(register27|0x80);
	recv = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	return(recv);
*/
}

// *************************************************************************
//   機能		： SPI_gyro_read
//   注意		： なし
//   メモ		： 初回実行
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.05			sato		新規
// *************************************************************************/
PUBLIC USHORT recv_spi_gyro(void)
{
	USHORT recv = 0;
	USHORT recv1;
	USHORT recv2;
	USHORT gyro_H = (0x47|0x80);	//register71
	USHORT gyro_L = (0x48|0x80);	//register72
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv1 = recv_spi(gyro_H);
	recv1 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv2 = recv_spi(gyro_L);
	recv2 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	RSPI0.SPSR.BYTE = 0xA0;
	
	recv = (recv1<<8)+(recv2&0xFF);
	
	return(recv);
		
}

// *************************************************************************
//   機能		： SPI_gyro_read
//   注意		： なし
//   メモ		： 初回実行
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.25			sato		新規
// *************************************************************************/
PUBLIC USHORT recv_spi_gyrooffset(void)
{
	USHORT recv = 0;
	USHORT recv1;
	USHORT recv2;
	USHORT gyro_H = (0x23|0x80);	//register23
	USHORT gyro_L = (0x24|0x80);	//register24
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv1 = recv_spi(gyro_H);
	recv1 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	PORTC.PODR.BIT.B4 = 0;
	TIME_waitFree(50);
	recv2 = recv_spi(gyro_L);
	recv2 = recv_spi(0x00);
	PORTC.PODR.BIT.B4 = 1;
	
	RSPI0.SPSR.BYTE = 0xA0;
	
	recv = (recv1<<8)+(recv2&0xFF);
	
	return(recv);
		
}


// *************************************************************************
//   機能		： DCMの回転方向をCW（時計回り）にする
//   注意		： なし
//   メモ		： なし
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2018.08.19			sato	回転方向は後でチェック	新規
// *************************************************************************/
PUBLIC void DCM_setDirCw( enDCM_ID en_id )
{
	/* 回転方向設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN = OFF;				// BIN1
	}
	else{							// 左
		DCM_L_IN = ON;			// AIN1

	}
}


// *************************************************************************
//   機能		： DCMの回転方向をCCW（反時計回り）にする
//   注意		： なし
//   メモ		： なし
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_setDirCcw( enDCM_ID en_id )
{
	/* 回転方向設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN = ON;			// BIN1
	}
	else{							// 左
		DCM_L_IN = OFF;				// AIN1
	}
}


// *************************************************************************
//   機能		： DCMを停止する
//   注意		： なし
//   メモ		： PWMのHI出力中に本関数を実行すると、ピンが100%出力状態なるため、関数内でピンをクリア（Lo）する。
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_stopMot( enDCM_ID en_id )
{
	/* 停止設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN = OFF;			// BIN1
		DCM_R_TIMER = OFF;			// タイマ停止
		DCM_R_TIORB = 1;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
	else{							// 左
		DCM_L_IN = OFF;			// AIN1
		DCM_L_TIMER = OFF;			// タイマ停止
		DCM_L_TIORB = 1;			// TIOCA 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
}


// *************************************************************************
//   機能		： DCMをブレーキングする
//   注意		： なし
//   メモ		： なし
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_brakeMot( enDCM_ID en_id )
{
	/* 停止設定 */
	if( en_id == DCM_R ){			// 右
		DCM_R_IN = ON;				// BIN1
		DCM_R_TIMER = OFF;			// タイマ停止
		DCM_R_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
	else{							// 左
		DCM_L_IN = ON;				// AIN1
		DCM_L_TIMER = OFF;			// タイマ停止
	    	DCM_L_TIORB = 1;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 0 出力
	}
}


// *************************************************************************
//   機能		： DCMを動作開始する
//   注意		： なし
//   メモ		： 動作開始前にPWMと回転方向を指定しておくこと
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_staMot( enDCM_ID en_id )
{
	/* タイマスタート */
	if( en_id == DCM_R ){			// 右
		DCM_R_TIORB = 2;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
		DCM_R_TIMER = ON;			// タイマ開始
	}
	else{							// 左
	    	DCM_L_TIORB = 2;			// TIOCB 端子の機能 : 初期出力は 0 出力。コンペアマッチで 1 出力
		DCM_L_TIMER = ON;			// タイマ開始
	}
}


// *************************************************************************
//   機能		： 全DCMを動作開始する
//   注意		： なし
//   メモ		： 動作開始前にPWMと回転方向を指定しておくこと
//   引数		： モータID
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_staMotAll( void )
{
	DCM_staMot(DCM_R);									// 右モータON
	DCM_staMot(DCM_L);									// 左モータON
}


// *************************************************************************
//   機能		： DCMのPWM-Dutyを設定する
//   注意		： 割り込み外から設定すると、ダブルバッファでないと歯抜けになる場合がある。
//   メモ		： 割り込みハンドラから実行すること。Duty0%の場合モータを停止させる（PWMにひげが出る）
//   引数		： モータID、？？？？？
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.03			外川			新規
// *************************************************************************/
PUBLIC void DCM_setPwmDuty( enDCM_ID en_id, USHORT us_duty10 )
{
	USHORT	us_cycle;							// 周期
	USHORT	us_onReg;							// 設定するON-duty
	
	/* PWM設定 */
	if( en_id == DCM_R ){				// 右
	
		if( 0 == us_duty10 ){			// Duty0%設定
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_R_TIMER = OFF;			// タイマ停止
			DCM_R_TCNT = 0;				// TCNT カウンタをクリア
			DCM_R_GRB = 5000;			// タイマ値変更
		    	DCM_R_TIORB = 6;			// TIOCB 端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
			DCM_R_TIMER = ON;			// タイマ開始
			us_duty10 = 1000;
		}
		else{
			us_cycle = DCM_R_GRA;		// 周期
			us_onReg = (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg 計算式
			DCM_R_TIMER = OFF;			// タイマ停止
			DCM_R_TCNT = 0;				// TCNT カウンタをクリア
			DCM_R_GRB = us_onReg;		// onDuty
			DCM_staMot( en_id );		// 回転開始
			printf("%f\r\n",us_cycle);
		}
	}
	else{								// 左

		if( 0 == us_duty10 ){			// Duty0%
			DCM_brakeMot( en_id );
		}
		else if( 1000 <= us_duty10 ){	// Duty100%

			DCM_L_TIMER = OFF;			// タイマ停止
			DCM_L_TCNT = 0;				// TCNT カウンタをクリア
			DCM_L_GRB = 5000;			// タイマ値変更
		    	DCM_L_TIORB = 6;			// TIOCB 端子の機能 : 初期出力は 1 出力。コンペアマッチで 1 出力
			DCM_L_TIMER = ON;			// タイマ開始
			us_duty10 = 1000;
		}
		else{
			us_cycle = DCM_L_GRA;		// 周期
			us_onReg = (USHORT)( (ULONG)us_cycle * (ULONG)us_duty10 / (ULONG)1000 );	// Duty2Reg 計算式
			DCM_L_TIMER = OFF;			// タイマ停止
			DCM_L_TCNT = 0;				// TCNT カウンタをクリア
			DCM_L_GRB = us_onReg;		// タイマ値変更
			DCM_staMot( en_id );		// 回転開始
			printf("%f\r\n",us_cycle);
		}
	}
}

// *************************************************************************
//   機能		： 制御を開始する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_sta( void )
{
	uc_CtrlFlag = TRUE;
}

// *************************************************************************
//   機能		： 制御を停止する
//   注意		： なし
//   メモ		： なし
//   引数		： なし
//   返り値		： なし
// **************************    履    歴    *******************************
// 		v1.0		2013.12.14			外川			新規
// *************************************************************************/
PUBLIC void CTRL_stop( void )
{
	uc_CtrlFlag = FALSE;
	DCM_brakeMot( DCM_R );		// ブレーキ
	DCM_brakeMot( DCM_L );		// ブレーキ
}
