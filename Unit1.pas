unit Unit1; // нахуесосил GPT 5.6 terra hight

interface

uses
  Windows, Messages, SysUtils, Variants, Classes, Graphics, Controls, Forms,
  Dialogs, StdCtrls, Gauges, ComCtrls, ExtCtrls, SyncObjs, Math,
  ComPort, IQDecoder;

const
  WM_SDR_FRAME    = WM_USER + 100;
  WM_SDR_LOG      = WM_USER + 101;
  WM_SDR_IQ_READY = WM_USER + 102;
  WM_SDR_STATE    = WM_USER + 103;

  SDR_STATE_IDLE  = 1;
  SDR_STATE_ERROR = 2;

  SDR_MIN_MHZ = 1842;
  SDR_MAX_MHZ = 2790;

  SDR_MAX_IQ_PAIRS = 16380;

type
  PByteArray = ^TByteArray;
  TByteArray = array[0..MaxInt - 1] of Byte;

  TRGB24 = packed record
    B, G, R: Byte;
  end;

  TSpectrumHeader = packed record
    Magic: Cardinal;
    FrameType: Byte;
    Reserved: Byte;
    StartHz: Double;
    StopHz: Double;
    BinsCount: Word;
    MinDb: Single;
    DbStep: Single;
    Timestamp: DWORD;
  end;

  TForm1 = class(TForm)
    Memo1: TMemo;
    Gauge1: TGauge;
    btnComportConnectDisconnect: TButton;
    tbGain: TTrackBar;
    pbScope: TPaintBox;
    edtCenterFreq: TEdit;
    cbSampleRate: TComboBox;
    dwnFreq: TButton;
    upFreq: TButton;
    btnFastSweep: TButton;
    ChkWateFall: TCheckBox;
    EndScanEdit: TEdit;
    StarScanEdit: TEdit;
    edtDeviceCOM: TEdit;
    btnStream: TButton;
    btnStop: TButton;
    cbFFT: TComboBox;
    edtFPS: TEdit;
    edtStep: TEdit;
    chkAGC: TCheckBox;
    cbWidth: TComboBox;
    btnChkRange: TButton;
    btnIQStream: TButton;
    cbIQDecim: TComboBox;
    chkPeakHold: TCheckBox;

    procedure FormCreate(Sender: TObject);
    procedure FormDestroy(Sender: TObject);
    procedure FormShow(Sender: TObject);

    procedure btnComportConnectDisconnectClick(Sender: TObject);
    procedure btnFastSweepClick(Sender: TObject);
    procedure btnStreamClick(Sender: TObject);
    procedure btnStopClick(Sender: TObject);
    procedure btnChkRangeClick(Sender: TObject);

    procedure cbSampleRateChange(Sender: TObject);
    procedure cbWidthChange(Sender: TObject);
    procedure tbGainChange(Sender: TObject);
    procedure chkAGCClick(Sender: TObject);

    procedure ChkWateFallClick(Sender: TObject);

    procedure upFreqClick(Sender: TObject);
    procedure dwnFreqClick(Sender: TObject);

    procedure pbScopePaint(Sender: TObject);
    procedure btnIQStreamClick(Sender: TObject);
    procedure chkPeakHoldClick(Sender: TObject);

  private
    FCS: TCriticalSection;

    { Native Windows event.
      0 = event was not created. }
    FIdleEvent: THandle;

    FBack: TBitmap;
    FWater: TBitmap;
    FWaterLUT: array[0..255] of TRGB24;

    FHdr: TSpectrumHeader;
    FDrawHdr: TSpectrumHeader;

    FSpec: AnsiString;
    FDrawSpec: AnsiString;

    FInvalidSpectrumHeaders: Cardinal;
    FReceivedSpectrumFrames: Cardinal;

    FRxThread: TThread;

    FStreaming: Boolean;
    FSweeping: Boolean;
    FIQStreaming: Boolean;
    FConfiguring: Boolean;

    FFrames: Integer;
    FFpsTick: DWORD;
    FFps: Single;

    FChkRunning: Boolean;
    FChkCancel: Boolean;

    FWaterFloorDb: Single;
    FWaterSpanDb: Single;

    FLastGainTick: DWORD;
    FLastPaintTick: DWORD;

    FSmMin: Single;
    FSmMax: Single;
    FSmSpec: array of Single;
    FSmStartHz: Double;
    FSmStopHz: Double;
    FSmInit: Boolean;
    FSmInit2: Boolean;

    FYTop: array of Integer;

    FGradR: array[0..255] of Byte;
    FGradG: array[0..255] of Byte;
    FGradB: array[0..255] of Byte;
    FGradA: array[0..255] of Byte;

    FLastIQ: TIQArray;
    FLastIQCount: Integer;
    FLastIQUs: Cardinal;

    FLastIQIsStream: Boolean;
    FLastIQSourceIndex: Int64;
    FLastIQDecimation: Cardinal;
    FLastIQSequence: Cardinal;
    FLastIQGapSamples: Int64;

    FIQNotifyPending: Longint;
    FIQLogTick: DWORD;

    FPeakHoldOn: Boolean;
    FPeakDb: array of Single;
    FPeakStartHz: Double;
    FPeakStopHz: Double;


FPeakHoldInit: Boolean;
FPeakHoldDb: array of Single;
FPeakHoldStartHz, FPeakHoldStopHz: Double;
FPeakHoldTick: DWORD;



FPeakLastHit: array of DWORD;
FPeakUpdateTick: DWORD;

FPeakHoldMs: DWORD;
FPeakDecayDbSec: Single;



    procedure ResetPeakHold;
    procedure UpdatePeakHoldRaw(const MinDb, DbStep: Single);

    procedure BuildGradientLUT;
    procedure BuildLUT;
    procedure EnsureExtraControls;

    procedure GetWaterfallColor(Level: Byte; out B, G, R: Byte);

    procedure ResetWaterfall;
    procedure AddWaterfallRow;

    procedure StoreFrame(const H: TSpectrumHeader;
      const Body: AnsiString);

    procedure WMSdrFrame(var Msg: TMessage); message WM_SDR_FRAME;
    procedure WMSdrLog(var Msg: TMessage); message WM_SDR_LOG;
    procedure WMSdrIQReady(var Msg: TMessage); message WM_SDR_IQ_READY;
    procedure WMSdrState(var Msg: TMessage); message WM_SDR_STATE;

    procedure StartReceiverThread;
    procedure StartStream;
    procedure StartSweep;
    procedure StartIQStream;
    procedure StopStreaming;

    procedure UpdateButtons;

    function GetFFTSize: Integer;
    function GetFPS: Integer;
    function GetStepMHz: Integer;
    function GetIQDecimation: Integer;

    function SdrWaitOK(TimeoutMs: Cardinal): Boolean;
    function WaitIdle(TimeoutMs: Cardinal): Boolean;

    function SendAndWaitOK(const Cmd: AnsiString;
      TimeoutMs: Cardinal): Boolean;

    function ConfigureReceiverFromUI: Boolean;

    procedure Log(const S: string);

  public
    procedure StoreIQ(const Pairs: TIQArray;
      Count: Integer; Us: Cardinal);

    procedure StoreIQStream(const Pairs: TIQArray;
      Count: Integer; Sequence: Cardinal;
      SourceIndex: Int64; Decimation: Cardinal;
      CaptureUs: Cardinal);
  end;

  TSdrRxThread = class(TThread)
  private
    FForm: TForm1;
    FBuf: AnsiString;
  protected
    procedure Execute; override;
  public
    constructor Create(AForm: TForm1);
  end;

var
  Form1: TForm1;

function SdrOpen(const Port: string): Boolean;
procedure SdrClose;
function SdrSendLine(const S: AnsiString): Boolean;
function SdrReadLine(out S: AnsiString;
  TimeoutMs: Cardinal): Boolean;

implementation

{$R *.dfm}


// =============================================================================
// COM helpers
// =============================================================================

function SdrOpen(const Port: string): Boolean;
begin
  Result := COM_Open(Port, 115200, 8, NOPARITY, ONESTOPBIT);

  if not Result then
    Exit;

  COM_SetBuffers(65536, 8192);

  { Non-blocking polling mode for synchronous ReadFile. }
  COM_SetTimeouts(MAXDWORD, 0, 1000, 0, 0);

  COM_SetDTR(True);
  COM_SetRTS(True);

  COM_Purge;
end;

procedure SdrClose;
begin
  COM_Close;
end;

function SdrSendLine(const S: AnsiString): Boolean;
var
  Data: AnsiString;
  Written: DWORD;
begin
  Result := False;

  if not COM_IsOpen then
    Exit;

  if Length(S) = 0 then
    Exit;

  Data := S + #13 + #10;

  Result :=
    COM_Write(Data[1], Length(Data), Written) and
    (Written = DWORD(Length(Data)));
end;

function SdrReadLine(out S: AnsiString;
  TimeoutMs: Cardinal): Boolean;
var
  T0: DWORD;
  B: Byte;
  ReadCnt: DWORD;
  Tmp: AnsiString;
begin
  S := '';
  Tmp := '';
  Result := False;

  T0 := GetTickCount;

  while GetTickCount - T0 < TimeoutMs do
  begin
    while COM_WaitingData > 0 do
    begin
      if not COM_Read(B, 1, ReadCnt) or
         (ReadCnt = 0) then
        Break;

      if B = 10 then
      begin
        S := Tmp;
        Result := True;
        Exit;
      end;

      if B <> 13 then
        Tmp := Tmp + AnsiChar(B);

      if Length(Tmp) > 512 then
      begin
        S := Tmp;
        Result := True;
        Exit;
      end;
    end;

    Sleep(2);
  end;
end;

// =============================================================================
// Form storage
// =============================================================================

procedure TForm1.StoreFrame(const H: TSpectrumHeader;
  const Body: AnsiString);
begin
  FCS.Enter;
  try
    FHdr := H;
    FSpec := Body;
  finally
    FCS.Leave;
  end;
end;

procedure TForm1.StoreIQ(const Pairs: TIQArray;
  Count: Integer; Us: Cardinal);
begin
  FCS.Enter;
  try
    FLastIQ := Pairs;
    FLastIQCount := Count;
    FLastIQUs := Us;

    FLastIQIsStream := False;
    FLastIQSequence := 0;
    FLastIQSourceIndex := 0;
    FLastIQDecimation := 0;
    FLastIQGapSamples := 0;
  finally
    FCS.Leave;
  end;
end;

procedure TForm1.StoreIQStream(const Pairs: TIQArray;
  Count: Integer; Sequence: Cardinal;
  SourceIndex: Int64; Decimation: Cardinal;
  CaptureUs: Cardinal);
var
  ExpectedNext: Int64;
begin
  FCS.Enter;
  try
    FLastIQGapSamples := 0;

    if FLastIQIsStream and
       (FLastIQDecimation <> 0) then
    begin
      ExpectedNext :=
        FLastIQSourceIndex +
        Int64(FLastIQCount) *
        Int64(FLastIQDecimation);

      if SourceIndex > ExpectedNext then
        FLastIQGapSamples := SourceIndex - ExpectedNext;
    end;

    FLastIQ := Pairs;
    FLastIQCount := Count;
    FLastIQUs := CaptureUs;

    FLastIQIsStream := True;
    FLastIQSequence := Sequence;
    FLastIQSourceIndex := SourceIndex;
    FLastIQDecimation := Decimation;
  finally
    FCS.Leave;
  end;
end;

// =============================================================================
// RX thread
// =============================================================================

constructor TSdrRxThread.Create(AForm: TForm1);
begin
  inherited Create(True);

  FreeOnTerminate := False;
  Priority := tpHigher;

  FForm := AForm;
  FBuf := '';
end;






// Helpers
function HdrFiniteF(const V: Single): Boolean;
begin
  Result := ((PCardinal(@V)^ shr 23) and $FF) <> $FF;
end;

function HdrFiniteD(const V: Double): Boolean;
begin
  Result := ((PCardinal(Cardinal(@V) + 4)^ shr 20) and $7FF) <> $7FF;
end;

function SpectrumHeaderValid(const H: TSpectrumHeader): Boolean;
begin
  Result := False;
  if H.Magic <> $5AA55AA5 then Exit;
  if not (H.FrameType in [1, 2]) then Exit;
  if (H.BinsCount < 16) or (H.BinsCount > 16384) then Exit;
  if not HdrFiniteD(H.StartHz) or not HdrFiniteD(H.StopHz) then Exit;
  if not HdrFiniteF(H.MinDb) or not HdrFiniteF(H.DbStep) then Exit;
  if (H.StartHz < 1.0E9) or (H.StopHz > 4.0E9) then Exit;
  if H.StopHz <= H.StartHz then Exit;
  if (H.MinDb < -200) or (H.MinDb > 50) then Exit;
  if (H.DbStep <= 0) or (H.DbStep > 5) then Exit;
  Result := True;
end;




procedure TSdrRxThread.Execute;
var
  Need, Got: DWORD;
  Buf: array[0..16383] of Byte;

  I, P, LineEnd, OldLen: Integer;

  H: TSpectrumHeader;
  BodyLen: Integer;

  MsgStr: PString;
  HeaderText: AnsiString;

  IQSamples, IQBytes, IQUs: Cardinal;
  IQPayload: AnsiString;

  ChunkPairs, AllPairs: TIQArray;
  ChunkOffset: Cardinal;
  BlockBytes: Cardinal;
  BlockCount: Word;
  BlockIndex: Int64;
  DecodedOK: Boolean;

  IQBSeq, IQBSourceLo, IQBSourceHi: Cardinal;
  IQBCount, IQBDecim, IQBBytes, IQBCaptureUs: Cardinal;
  IQBSourceIndex: Int64;
  IQBPayload: AnsiString;
    DATAIQR_IsBlocks, NeedMore: Boolean;
  DATAIQR_NBlocks: Cardinal;
  TmpOff, Bi: Integer;

  function IsSpace(C: AnsiChar): Boolean;
  begin
    Result := (C = ' ') or (C = #9);
  end;

  function FindLF(const S: AnsiString): Integer;
  var
    K: Integer;
  begin
    Result := 0;

    for K := 1 to Length(S) do
      if S[K] = #10 then
      begin
        Result := K;
        Exit;
      end;
  end;

  function StartsWith(const S, Prefix: AnsiString): Boolean;
  begin
    Result :=
      (Length(S) >= Length(Prefix)) and
      (Copy(S, 1, Length(Prefix)) = Prefix);
  end;

  function MagicAt(const S: AnsiString;
    Offset: Integer): Boolean;
  begin
    Result :=
      (Offset >= 1) and
      (Offset + 3 <= Length(S)) and
      (Byte(S[Offset]) = $A5) and
      (Byte(S[Offset + 1]) = $5A) and
      (Byte(S[Offset + 2]) = $A5) and
      (Byte(S[Offset + 3]) = $5A);
  end;

  function FindMagic(const S: AnsiString): Integer;
  var
    K: Integer;
  begin
    Result := 0;

    for K := 1 to Length(S) - 3 do
      if MagicAt(S, K) then
      begin
        Result := K;
        Exit;
      end;
  end;

  function NextToken(var S: AnsiString;
    out Token: AnsiString): Boolean;
  var
    K: Integer;
  begin
    Result := False;
    Token := '';

    while (Length(S) > 0) and IsSpace(S[1]) do
      Delete(S, 1, 1);

    if Length(S) = 0 then
      Exit;

    K := 1;

    while (K <= Length(S)) and
          not IsSpace(S[K]) do
      Inc(K);

    Token := Copy(S, 1, K - 1);
    Delete(S, 1, K - 1);

    while (Length(S) > 0) and IsSpace(S[1]) do
      Delete(S, 1, 1);

    Result := Token <> '';
  end;

  function ParseUInt32Dec(const S: AnsiString;
    out Value: Cardinal): Boolean;
  const
    MAX_U32: Int64 = 4294967295;
  var
    K: Integer;
    V: Int64;
    D: Integer;
  begin
    Result := False;
    Value := 0;

    if Length(S) = 0 then
      Exit;

    V := 0;

    for K := 1 to Length(S) do
    begin
      if (S[K] < '0') or
         (S[K] > '9') then
        Exit;

      D := Ord(S[K]) - Ord('0');

      V := V * 10 + D;

      if V > MAX_U32 then
        Exit;
    end;

    Value := Cardinal(V);
    Result := True;
  end;


   function ParseDataIQRHeader(const S: AnsiString;
    out Samples, Second, CaptureUs: Cardinal; out IsBlocks: Boolean): Boolean;
  var
    T, Token: AnsiString;
  begin
    Result := False;
    Samples := 0; Second := 0; CaptureUs := 0; IsBlocks := False;
    if not StartsWith(S, 'DATA_IQR ') then Exit;
    T := Copy(S, 10, MaxInt);
    if not NextToken(T, Token) then Exit;
    if not ParseUInt32Dec(Token, Samples) then Exit;
    if not NextToken(T, Token) then Exit;
    if not ParseUInt32Dec(Token, Second) then Exit;
    if not NextToken(T, Token) then Exit;
    if not ParseUInt32Dec(Token, CaptureUs) then Exit;
    if Trim(T) <> '' then Exit;
    if (Samples = 0) or (Samples > SDR_MAX_IQ_PAIRS) then Exit;
    // v10: second = nblocks 1..16, v9: second = total_bytes >600
    if Second <= 16 then
    begin
      IsBlocks := True;
      if Second <> ((Samples + 1023) div 1024) then Exit;
    end
    else
    begin
      IsBlocks := False;
      if (Second = 0) or (Second > 131072) then Exit;
    end;
    Result := True;
  end;


  function ParseIQBHeader(const S: AnsiString;
    out Seq, SourceLo, SourceHi, Count, Decim,
      Bytes, CaptureUs: Cardinal): Boolean;
  var
    T, Token: AnsiString;
    Values: array[0..6] of Cardinal;
    K: Integer;
  begin
    Result := False;

    Seq := 0;
    SourceLo := 0;
    SourceHi := 0;
    Count := 0;
    Decim := 0;
    Bytes := 0;
    CaptureUs := 0;

    if not StartsWith(S, 'IQB ') then
      Exit;

    T := Copy(S, 5, MaxInt);

    for K := 0 to 6 do
    begin
      if not NextToken(T, Token) then
        Exit;

      if not ParseUInt32Dec(Token, Values[K]) then
        Exit;
    end;

    if Trim(T) <> '' then
      Exit;

    Seq := Values[0];
    SourceLo := Values[1];
    SourceHi := Values[2];
    Count := Values[3];
    Decim := Values[4];
    Bytes := Values[5];
    CaptureUs := Values[6];

    if (Count = 0) or
       (Count > 1024) then
      Exit;

    if (Decim <> 256) and
       (Decim <> 1024) then
      Exit;

    if (Bytes < 32) or
       (Bytes > 4096) then
      Exit;

    Result := True;
  end;

  procedure PostLog(const S: AnsiString);
  begin
    New(MsgStr);
    MsgStr^ := string(S);

    if FForm.HandleAllocated then
      PostMessage(FForm.Handle,
        WM_SDR_LOG, Integer(MsgStr), 0)
    else
      Dispose(MsgStr);
  end;

  procedure PostState(Code: Integer);
  begin
    if FForm.HandleAllocated then
      PostMessage(FForm.Handle,
        WM_SDR_STATE, Code, 0);
  end;

  procedure PostIQReady;
  begin
    if InterlockedExchange(FForm.FIQNotifyPending, 1) = 0 then
    begin
      if FForm.HandleAllocated then
        PostMessage(FForm.Handle,
          WM_SDR_IQ_READY, 0, 0)
      else
        InterlockedExchange(FForm.FIQNotifyPending, 0);
    end;
  end;


  procedure ProcessTextLine(const Raw: AnsiString);
  var
    K, Cut: Integer;
    Text: AnsiString;
  begin
    Cut := 0;
    for K := 1 to Length(Raw) do
      if (Byte(Raw[K]) < 32) or (Byte(Raw[K]) > 126) then
        Cut := K;

    if Cut > 0 then
      Text := Trim(Copy(Raw, Cut + 1, MaxInt))
    else
      Text := Raw;

    if Text = '' then
      Exit;

    if Text = '#IDLE' then
    begin
      if FForm.FIdleEvent <> 0 then
        Windows.SetEvent(FForm.FIdleEvent);

      PostState(SDR_STATE_IDLE);
    end
    else if StartsWith(Text, '#STREAM_ERR') or
            StartsWith(Text, '#IQSTREAM_ERR') or
            StartsWith(Text, '#SWEEP_ERR') or
            StartsWith(Text, '#DETECT_ERR') then
      PostState(SDR_STATE_ERROR);

    PostLog(Text);
  end;


begin
  FBuf := '';

  while not Terminated do
  begin
    if not COM_IsOpen then
    begin
      Sleep(50);
      Continue;
    end;

    Need := COM_WaitingData;

    if Need = 0 then
    begin
      Sleep(2);
      Continue;
    end;

    if Need > SizeOf(Buf) then
      Need := SizeOf(Buf);

    if (not COM_Read(Buf[0], Need, Got)) or
       (Got = 0) then
    begin
      Sleep(2);
      Continue;
    end;

    OldLen := Length(FBuf);
    SetLength(FBuf, OldLen + Integer(Got));

    Move(Buf[0], FBuf[OldLen + 1], Got);

    while not Terminated and
          (Length(FBuf) > 0) do
    begin
      { IQSTREAM:
        IQB <seq> <source_lo> <source_hi> <pairs>
            <decimation> <bytes> <capture_us>\n
        <IQR binary record> }

      if StartsWith(FBuf, 'IQB ') then
      begin
        LineEnd := FindLF(FBuf);

        if LineEnd = 0 then
          Break;

        HeaderText := Trim(Copy(FBuf, 1, LineEnd));

        if not ParseIQBHeader(
          HeaderText,
          IQBSeq,
          IQBSourceLo,
          IQBSourceHi,
          IQBCount,
          IQBDecim,
          IQBBytes,
          IQBCaptureUs) then
        begin
          Delete(FBuf, 1, LineEnd);
          PostLog('IQB bad header: ' + HeaderText);
          Continue;
        end;

        if Length(FBuf) <
           LineEnd + Integer(IQBBytes) then
          Break;

        IQBPayload :=
          Copy(FBuf, LineEnd + 1, Integer(IQBBytes));

        Delete(FBuf, 1,
          LineEnd + Integer(IQBBytes));

        IQBSourceIndex :=
          (Int64(IQBSourceHi) shl 32) +
          Int64(IQBSourceLo);

        if not IQRecordSize(
          Pointer(PAnsiChar(IQBPayload)),
          IQBBytes,
          BlockBytes,
          BlockCount,
          BlockIndex) then
        begin
          PostLog(Format(
            'IQB invalid IQR record seq=%u',
            [IQBSeq]));
          Continue;
        end;

        if (BlockBytes <> IQBBytes) or
           (BlockCount <> IQBCount) or
           (BlockIndex <> IQBSourceIndex) then
        begin
          PostLog(Format(
            'IQB header/IQR mismatch seq=%u',
            [IQBSeq]));
          Continue;
        end;

        SetLength(ChunkPairs, 0);

        if not DecodeIQRecord(
          Pointer(PAnsiChar(IQBPayload)),
          IQBBytes,
          ChunkPairs) then
        begin
          PostLog(Format(
            'IQB decode error seq=%u bytes=%u',
            [IQBSeq, IQBBytes]));
          Continue;
        end;

        if Length(ChunkPairs) <> Integer(IQBCount) then
        begin
          PostLog(Format(
            'IQB count mismatch seq=%u header=%u decoded=%d',
            [IQBSeq, IQBCount, Length(ChunkPairs)]));
          Continue;
        end;

        FForm.StoreIQStream(
          ChunkPairs,
          Length(ChunkPairs),
          IQBSeq,
          IQBSourceIndex,
          IQBDecim,
          IQBCaptureUs);

        PostIQReady;
        Continue;
      end;

      { CAPIQC:
        DATA_IQR <pairs> <total-bytes> <capture-us>\n
        <one or more IQR binary records> }



      if StartsWith(FBuf, 'DATA_IQR ') then
      begin
        LineEnd := FindLF(FBuf);
        if LineEnd = 0 then Break;
        HeaderText := Trim(Copy(FBuf, 1, LineEnd));
        if not ParseDataIQRHeader(HeaderText, IQSamples, IQBytes, IQUs, DATAIQR_IsBlocks) then
        begin
          Delete(FBuf, 1, LineEnd);
          PostLog('DATA_IQR bad header: ' + HeaderText);
          Continue;
        end;

        if DATAIQR_IsBlocks then
        begin
          DATAIQR_NBlocks := IQBytes;
          TmpOff := LineEnd + 1;
          SetLength(AllPairs, 0);
          DecodedOK := True;
          NeedMore := False;
          for Bi := 1 to Integer(DATAIQR_NBlocks) do
          begin
            if Length(FBuf) < TmpOff + 32 - 1 then begin NeedMore := True; Break; end;
            if not IQRecordSize(Pointer(@FBuf[TmpOff]), Length(FBuf)-TmpOff+1, BlockBytes, BlockCount, BlockIndex) then
            begin
              if (BlockBytes > 0) and (TmpOff + Integer(BlockBytes) - 1 > Length(FBuf)) then NeedMore := True
              else DecodedOK := False;
              Break;
            end;
            if (BlockBytes = 0) or (TmpOff + Integer(BlockBytes) - 1 > Length(FBuf)) then begin NeedMore := True; Break; end;
            SetLength(ChunkPairs, 0);
            if not DecodeIQRecord(Pointer(@FBuf[TmpOff]), BlockBytes, ChunkPairs) then begin DecodedOK := False; Break; end;
            if Length(ChunkPairs) <> Integer(BlockCount) then begin DecodedOK := False; Break; end;
            OldLen := Length(AllPairs);
            SetLength(AllPairs, OldLen + Length(ChunkPairs));
            if Length(ChunkPairs) > 0 then
              Move(ChunkPairs[0], AllPairs[OldLen], Length(ChunkPairs) * SizeOf(TIQPair));
            Inc(TmpOff, BlockBytes);
          end;
          if NeedMore then Break; // FBuf не трогаем, ждем догрузки
          if (not DecodedOK) or (Length(AllPairs) <> Integer(IQSamples)) then
          begin
            if TmpOff > LineEnd + 1 then Delete(FBuf, 1, TmpOff-1)
            else Delete(FBuf, 1, LineEnd);
            PostLog(Format('DATA_IQR v10 rejected expected=%u decoded=%d blocks=%u',
              [IQSamples, Length(AllPairs), DATAIQR_NBlocks]));
            Continue;
          end;
          Delete(FBuf, 1, TmpOff-1);
          FForm.StoreIQ(AllPairs, Length(AllPairs), IQUs);
          PostIQReady;
          Continue;
        end
        else
        begin
          // legacy total_bytes
          if (IQBytes = 0) or (IQBytes > 131072) then
          begin
            Delete(FBuf, 1, LineEnd);
            PostLog('DATA_IQR invalid length');
            Continue;
          end;
          if Length(FBuf) < LineEnd + Integer(IQBytes) then Break;
          IQPayload := Copy(FBuf, LineEnd + 1, Integer(IQBytes));
          Delete(FBuf, 1, LineEnd + Integer(IQBytes));
          ChunkOffset := 0;
          SetLength(AllPairs, 0);
          DecodedOK := True;
          while ChunkOffset < IQBytes do
          begin
            if not IQRecordSize(Pointer(Cardinal(PAnsiChar(IQPayload)) + ChunkOffset),
              IQBytes - ChunkOffset, BlockBytes, BlockCount, BlockIndex) then begin DecodedOK := False; Break; end;
            if (BlockBytes = 0) or (ChunkOffset + BlockBytes > IQBytes) then begin DecodedOK := False; Break; end;
            SetLength(ChunkPairs, 0);
            if not DecodeIQRecord(Pointer(Cardinal(PAnsiChar(IQPayload)) + ChunkOffset),
              BlockBytes, ChunkPairs) then begin DecodedOK := False; Break; end;
            if Length(ChunkPairs) <> Integer(BlockCount) then begin DecodedOK := False; Break; end;
            OldLen := Length(AllPairs);
            SetLength(AllPairs, OldLen + Length(ChunkPairs));
            if Length(ChunkPairs) > 0 then
              Move(ChunkPairs[0], AllPairs[OldLen], Length(ChunkPairs) * SizeOf(TIQPair));
            Inc(ChunkOffset, BlockBytes);
          end;
          if (not DecodedOK) or (ChunkOffset <> IQBytes) or (Length(AllPairs) <> Integer(IQSamples)) then
          begin
            PostLog(Format('DATA_IQR rejected expected=%u decoded=%d bytes=%u parsed=%u',
              [IQSamples, Length(AllPairs), IQBytes, ChunkOffset]));
            Continue;
          end;
          FForm.StoreIQ(AllPairs, Length(AllPairs), IQUs);
          PostIQReady;
          Continue;
        end;
      end;








      { Spectrum frame has priority over text LF. }
      P := FindMagic(FBuf);
      LineEnd := FindLF(FBuf);

      if (P > 0) and
         ((LineEnd = 0) or (P < LineEnd)) then
      begin
        if P > 1 then
        begin
          Delete(FBuf, 1, P - 1);
          Continue;
        end;

        if Length(FBuf) < SizeOf(TSpectrumHeader) then
          Break;

        Move(FBuf[1], H, SizeOf(TSpectrumHeader));

      if not SpectrumHeaderValid(H) then
        begin
          Delete(FBuf, 1, 1);
          Continue;
        end;



        BodyLen :=
          SizeOf(TSpectrumHeader) + H.BinsCount;

        if Length(FBuf) < BodyLen then
          Break;

        FForm.StoreFrame(H,
          Copy(FBuf,
            SizeOf(TSpectrumHeader) + 1,
            H.BinsCount));

        Delete(FBuf, 1, BodyLen);

        if FForm.HandleAllocated then
          PostMessage(FForm.Handle,
            WM_SDR_FRAME, 0, 0);

        Continue;
      end;

      { Ordinary ASCII text line. }
      if LineEnd > 0 then
      begin
        HeaderText := Trim(Copy(FBuf, 1, LineEnd));
        Delete(FBuf, 1, LineEnd);

        ProcessTextLine(HeaderText);
        Continue;
      end;

      { Bad/incomplete input safety limit. }
      if Length(FBuf) > 150000 then
      begin
        SetLength(FBuf, 0);
        PostLog('RX buffer reset: oversized undecoded data');
      end;

      Break;
    end;
  end;
end;

// =============================================================================
// Log / notifications
// =============================================================================

procedure TForm1.Log(const S: string);
begin
  Memo1.Lines.Add(S);

  while Memo1.Lines.Count > 500 do
    Memo1.Lines.Delete(0);
end;

procedure TForm1.WMSdrLog(var Msg: TMessage);
var
  PStr: PString;
begin
  PStr := PString(Msg.WParam);

  if PStr <> nil then
  begin
    Log('<< ' + PStr^);
    Dispose(PStr);
  end;
end;

procedure TForm1.WMSdrState(var Msg: TMessage);
begin
  case Integer(Msg.WParam) of
    SDR_STATE_IDLE,
    SDR_STATE_ERROR:
      begin
        FStreaming := False;
        FSweeping := False;
        FIQStreaming := False;
        UpdateButtons;
      end;
  end;
end;

procedure TForm1.WMSdrIQReady(var Msg: TMessage);
var
  IsStream: Boolean;
  Sequence: Cardinal;
  SourceIndex: Int64;
  Decimation: Cardinal;
  Count: Integer;
  CaptureUs: Cardinal;
  GapSamples: Int64;
  NowTick: DWORD;
  GapText: string;
begin
  InterlockedExchange(FIQNotifyPending, 0);

  FCS.Enter;
  try
    IsStream := FLastIQIsStream;
    Sequence := FLastIQSequence;
    SourceIndex := FLastIQSourceIndex;
    Decimation := FLastIQDecimation;
    Count := FLastIQCount;
    CaptureUs := FLastIQUs;
    GapSamples := FLastIQGapSamples;
  finally
    FCS.Leave;
  end;

  if not IsStream then
  begin
    Log(Format(
      '<< CAPIQC IQ pairs=%d capture=%u us',
      [Count, CaptureUs]));
    Exit;
  end;

  NowTick := GetTickCount;

  { Avoid flooding TMemo during IQSTREAM. }
  if NowTick - FIQLogTick < 250 then
    Exit;

  FIQLogTick := NowTick;

  if GapSamples > 0 then
    GapText :=
      ' gap=' + IntToStr(GapSamples) +
      ' source-pairs'
  else
    GapText := '';

  Log(
    '<< IQ seq=' + IntToStr(Sequence) +
    ' source=' + IntToStr(SourceIndex) +
    ' pairs=' + IntToStr(Count) +
    ' decim=' + IntToStr(Decimation) +
    ' capture=' + IntToStr(CaptureUs) +
    ' us' + GapText);
end;

// =============================================================================
// Waterfall
// =============================================================================

procedure TForm1.GetWaterfallColor(Level: Byte;
  out B, G, R: Byte);
var
  T: Integer;
begin
  if Level < 30 then
  begin
    R := 0;
    G := 0;
    B := Level * 2;
  end
  else if Level < 80 then
  begin
    T := (Level - 30) * 5;
    if T > 255 then T := 255;

    R := 0;
    G := 0;
    B := 60 + T;
  end
  else if Level < 130 then
  begin
    T := (Level - 80) * 5;
    if T > 255 then T := 255;

    R := 0;
    G := T;
    B := 255 - T;
  end
  else if Level < 180 then
  begin
    T := (Level - 130) * 5;
    if T > 255 then T := 255;

    R := T;
    G := 255;
    B := 0;
  end
  else if Level < 220 then
  begin
    T := (Level - 180) * 6;
    if T > 255 then T := 255;

    R := 255;
    G := 255 - T div 2;
    B := 0;
  end
  else
  begin
    T := (Level - 220) * 5;
    if T > 255 then T := 255;

    R := 255;
    G := 0;
    B := 0;
  end;
end;

procedure TForm1.BuildLUT;
var
  I: Integer;
  B, G, R: Byte;
begin
  for I := 0 to 255 do
  begin
    GetWaterfallColor(I, B, G, R);

    FWaterLUT[I].B := B;
    FWaterLUT[I].G := G;
    FWaterLUT[I].R := R;
  end;
end;

procedure TForm1.BuildGradientLUT;
var
  I: Integer;
  T, TS: Double;
begin
  for I := 0 to 255 do
  begin
    T := I / 255.0;
    TS := Sqrt(T);

    FGradR[I] := Byte(Round(6 + 70 * TS));
    FGradG[I] := Byte(Round(20 + 180 * TS));
    FGradB[I] := Byte(Round(6 + 35 * TS));
    FGradA[I] := Byte(Round(22 + 85 * TS));
  end;
end;

procedure TForm1.ResetWaterfall;
var
  W, H: Integer;
begin
  if FWater = nil then
    Exit;

  W := pbScope.Width;
  if W < 64 then W := 64;

  H := 300;

  if pbScope.Height > 100 then
    H := pbScope.Height * 40 div 100;

  if H < 80 then H := 80;
  if H > 600 then H := 600;

  if (FWater.Width <> W) or
     (FWater.Height <> H) then
  begin
    FWater.Width := W;
    FWater.Height := H;
    FWater.PixelFormat := pf24bit;
  end;

  FWater.Canvas.Brush.Color := clBlack;
  FWater.Canvas.FillRect(Rect(0, 0, W, H));
end;

procedure TForm1.AddWaterfallRow;
var
  W, N, X, C0, C1, J: Integer;
  Row: PByteArray;

  DbMax, Norm: Single;
  Level: Integer;

  FrameStart, FrameStop, BinHz: Double;
  Fx0, Fx1: Double;
begin
  if (FWater = nil) or
     (Length(FSmSpec) = 0) then
    Exit;

  W := pbScope.Width;

  if W < 64 then
    Exit;

  N := Length(FSmSpec);

  FrameStart := FSmStartHz;
  FrameStop := FSmStopHz;

  if (N < 16) or
     (FrameStop <= FrameStart) then
    Exit;

  BinHz := (FrameStop - FrameStart) / N;

  if BinHz <= 0 then
    Exit;

  if FWater.Width <> W then
    ResetWaterfall;

  for J := FWater.Height - 1 downto 1 do
    Move(FWater.ScanLine[J - 1]^,
      FWater.ScanLine[J]^,
      W * 3);

  Row := FWater.ScanLine[0];

  for X := 0 to W - 1 do
  begin
    Fx0 :=
      FrameStart +
      (FrameStop - FrameStart) * X / W;

    Fx1 :=
      FrameStart +
      (FrameStop - FrameStart) * (X + 1) / W;

    C0 := Trunc((Fx0 - FrameStart) / BinHz);
    C1 := Trunc((Fx1 - FrameStart - 0.001) / BinHz);

    if C1 < C0 then C1 := C0;
    if C0 < 0 then C0 := 0;
    if C1 >= N then C1 := N - 1;

    DbMax := -200.0;

    for J := C0 to C1 do
      if FSmSpec[J] > DbMax then
        DbMax := FSmSpec[J];



    { Confirmed working default scale:
      -100 dB floor, 90 dB visible range. }
    if DbMax <= FWaterFloorDb then
      Level := 0
    else if DbMax >= FWaterFloorDb + FWaterSpanDb then
      Level := 255
    else
    begin
      Norm :=
        (DbMax - FWaterFloorDb) /
        FWaterSpanDb;

      Norm := Norm * Norm * (3.0 - 2.0 * Norm);

      Level := Round(Norm * 255.0);
    end;


    if Level < 0 then Level := 0;
    if Level > 255 then Level := 255;


    Row^[X * 3 + 0] := FWaterLUT[Level].B;
    Row^[X * 3 + 1] := FWaterLUT[Level].G;
    Row^[X * 3 + 2] := FWaterLUT[Level].R;
  end;
end;




procedure TForm1.chkPeakHoldClick(Sender: TObject);
begin
  FPeakHoldOn := chkPeakHold.Checked;
  ResetPeakHold;
  if Assigned(pbScope) then pbScope.Invalidate;
end;

procedure TForm1.ResetPeakHold;
begin
  SetLength(FPeakHoldDb, 0);
  SetLength(FPeakLastHit, 0);

  FPeakHoldInit := False;
  FPeakHoldStartHz := 0;
  FPeakHoldStopHz := 0;
  FPeakUpdateTick := GetTickCount;
end;

procedure TForm1.UpdatePeakHoldRaw(
  const MinDb, DbStep: Single);
const
  DB_LO = -150.0;
  DB_HI = 20.0;
var
  I, N: Integer;
  RawDb: Single;
  NowTick, Dt: DWORD;
  Decay: Single;
  NeedReset: Boolean;
begin
  if not FPeakHoldOn then
    Exit;

  N := Length(FDrawSpec);

  if N < 16 then
    Exit;

  NowTick := GetTickCount;
  Dt := NowTick - FPeakUpdateTick;

  if Dt > 1000 then
    Dt := 1000;

  FPeakUpdateTick := NowTick;
  Decay := FPeakDecayDbSec * Dt / 1000.0;

  NeedReset :=
    (not FPeakHoldInit) or
    (Length(FPeakHoldDb) <> N) or
    (Length(FPeakLastHit) <> N) or
    (Abs(FDrawHdr.StartHz - FPeakHoldStartHz) > 0.5) or
    (Abs(FDrawHdr.StopHz - FPeakHoldStopHz) > 0.5);

  if NeedReset then
  begin
    SetLength(FPeakHoldDb, N);
    SetLength(FPeakLastHit, N);

    for I := 0 to N - 1 do
    begin
      RawDb :=
        MinDb +
        Byte(FDrawSpec[I + 1]) * DbStep;

      if RawDb < DB_LO then RawDb := DB_LO;
      if RawDb > DB_HI then RawDb := DB_HI;

      FPeakHoldDb[I] := RawDb;
      FPeakLastHit[I] := NowTick;
    end;

    FPeakHoldStartHz := FDrawHdr.StartHz;
    FPeakHoldStopHz := FDrawHdr.StopHz;
    FPeakHoldInit := True;
    Exit;
  end;

  for I := 0 to N - 1 do
  begin
    RawDb :=
      MinDb +
      Byte(FDrawSpec[I + 1]) * DbStep;

    if RawDb < DB_LO then RawDb := DB_LO;
    if RawDb > DB_HI then RawDb := DB_HI;

    if RawDb >= FPeakHoldDb[I] then
    begin
      { Мгновенная атака. }
      FPeakHoldDb[I] := RawDb;
      FPeakLastHit[I] := NowTick;
    end
    else if DWORD(NowTick - FPeakLastHit[I]) >= FPeakHoldMs then
    begin
      { Плавный спад после выдержки. }
      FPeakHoldDb[I] := FPeakHoldDb[I] - Decay;

      if FPeakHoldDb[I] < RawDb then
        FPeakHoldDb[I] := RawDb;

      if FPeakHoldDb[I] < DB_LO then
        FPeakHoldDb[I] := DB_LO;
    end;
  end;

  FPeakHoldTick := NowTick;
end;


procedure TForm1.WMSdrFrame(var Msg: TMessage);
const
  DB_LO = -150.0;
  DB_HI = 20.0;
var
  Dt, NowTick: DWORD;
  I, N, P: Integer;
  Target, Cur: Single;
  MinDb, DbStep: Single;
  HdrOk, NeedReset: Boolean;


   function FiniteF(const V: Single): Boolean;
  var
    U: Cardinal;
  begin
    U := PCardinal(@V)^;
    { Экспонента $FF = NaN или Inf. Проверяем биты, не FPU. }
    Result := ((U shr 23) and $FF) <> $FF;
    if Result then
      Result := Abs(V) <= 1.0E30;
  end;

  function FiniteD(const V: Double): Boolean;
  var
    Hi: Cardinal;
  begin
    Hi := PCardinal(Cardinal(@V) + 4)^;
    { Экспонента $7FF = NaN или Inf. }
    Result := ((Hi shr 20) and $7FF) <> $7FF;
    if Result then
      Result := Abs(V) <= 1.0E30;
  end;


  function SanitizeDb(const V: Single): Single;
  begin
    if not FiniteF(V) then
      Result := -120.0
    else if V < DB_LO then
      Result := DB_LO
    else if V > DB_HI then
      Result := DB_HI
    else
      Result := V;
  end;

begin
  FCS.Enter;
  try
    FDrawHdr := FHdr;
    FDrawSpec := FSpec;
  finally
    FCS.Leave;
  end;

  N := Length(FDrawSpec);

  { -------- Жёсткая проверка кадра до любых вычислений -------- }
  HdrOk :=
    (N >= 16) and
    (N <= 16384) and
    (FDrawHdr.BinsCount >= 16) and
    (FDrawHdr.BinsCount <= 16384) and
    (N = Integer(FDrawHdr.BinsCount)) and
    (FDrawHdr.FrameType in [1, 2]) and
    FiniteD(FDrawHdr.StartHz) and
    FiniteD(FDrawHdr.StopHz) and
    (FDrawHdr.StartHz > 1.0E9) and
    (FDrawHdr.StopHz < 4.0E9) and
    (FDrawHdr.StopHz > FDrawHdr.StartHz) and
    FiniteF(FDrawHdr.MinDb) and
    FiniteF(FDrawHdr.DbStep) and
    (FDrawHdr.DbStep > 0.0) and
    (FDrawHdr.DbStep <= 5.0) and
    (FDrawHdr.MinDb >= -200.0) and
    (FDrawHdr.MinDb <= 50.0);

  if not HdrOk then
  begin
    { Отбрасываем испорченный кадр; если шкала уже "уехала" — сброс }
    if (not FiniteF(FSmMax)) or (not FiniteF(FSmMin)) or
       (FSmMax > DB_HI) or (FSmMin < -160.0) or
       ((FSmMax - FSmMin) > 100.0) then
    begin
      FSmInit := False;
      FSmInit2 := False;
      SetLength(FSmSpec, 0);
    end;
    Exit;
  end;

  MinDb := FDrawHdr.MinDb;
  DbStep := FDrawHdr.DbStep;

  { -------- FPS / заголовок окна -------- }
  Inc(FFrames);

  NowTick := GetTickCount;
  Dt := NowTick - FFpsTick;

  if Dt >= 1000 then
  begin
    FFps := FFrames * 1000.0 / Dt;
    FFrames := 0;
    FFpsTick := NowTick;

    P := Round(FFps * 5);
    if P < 0 then P := 0;
    if P > 100 then P := 100;
    Gauge1.Progress := P;

    Caption := Format(
      'ESP-S3 SDR %.1f-%.1f MHz bins=%d %.1f fps type=%d',
      [FDrawHdr.StartHz / 1e6,
       FDrawHdr.StopHz / 1e6,
       FDrawHdr.BinsCount,
       FFps,
       FDrawHdr.FrameType]);
  end;

  { -------- Сглаживание спектра -------- }
  try
    NeedReset :=
      (not FSmInit2) or
      (Length(FSmSpec) <> N) or
      (Abs(FDrawHdr.StartHz - FSmStartHz) > 0.5) or
      (Abs(FDrawHdr.StopHz - FSmStopHz) > 0.5);

    { Если массив уже отравлен прошлым кадром — пересоздаём }
    if not NeedReset then
      for I := 0 to N - 1 do
      begin
        Cur := FSmSpec[I];
        if (not FiniteF(Cur)) or (Cur < DB_LO - 1) or (Cur > DB_HI + 1) then
        begin
          NeedReset := True;
          Break;
        end;
      end;

    if NeedReset then
    begin
      SetLength(FSmSpec, N);

      FSmStartHz := FDrawHdr.StartHz;
      FSmStopHz := FDrawHdr.StopHz;

      { AnsiString индексируется с 1: допустимо 1..N }
      for I := 0 to N - 1 do
        FSmSpec[I] := SanitizeDb(MinDb + Byte(FDrawSpec[I + 1]) * DbStep);

      FSmInit2 := True;
      FSmInit := False;   { пересчитать автошкалу заново }
    end
    else
    begin
      for I := 0 to N - 1 do
      begin
        Target := SanitizeDb(MinDb + Byte(FDrawSpec[I + 1]) * DbStep);
        Cur := FSmSpec[I];

        if Target > Cur then
          Cur := Cur + (Target - Cur) * 0.75
        else
          Cur := Cur + (Target - Cur) * 0.35;

        FSmSpec[I] := SanitizeDb(Cur);
      end;
    end;
  except
    { Любое FPU-исключение: сброс, следующий кадр восстановит картину }
    FSmInit := False;
    FSmInit2 := False;
    SetLength(FSmSpec, 0);
    Exit;
  end;


  { -------- Peak Hold из сырого кадра -------- }


UpdatePeakHoldRaw(MinDb, DbStep);


  { -------- Ограничение частоты перерисовки -------- }
  if NowTick - FLastPaintTick < 33 then
    Exit;

  FLastPaintTick := NowTick;

  if ChkWateFall.Checked then
    AddWaterfallRow;

  pbScope.Invalidate;
end;
// =============================================================================
// UI controls and values
// =============================================================================

procedure TForm1.EnsureExtraControls;
begin
  if not Assigned(btnStream) then
  begin
    btnStream := TButton.Create(Self);
    btnStream.Parent := Self;
    btnStream.Name := 'btnStream';
    btnStream.Caption := 'Stream';
    btnStream.Width := btnFastSweep.Width;
    btnStream.Left := btnFastSweep.Left;
    btnStream.Top := btnFastSweep.Top + 30;
    btnStream.OnClick := btnStreamClick;
  end;

  if not Assigned(btnStop) then
  begin
    btnStop := TButton.Create(Self);
    btnStop.Parent := Self;
    btnStop.Name := 'btnStop';
    btnStop.Caption := 'Stop';
    btnStop.Width := btnStream.Width;
    btnStop.Left := btnStream.Left + btnStream.Width + 6;
    btnStop.Top := btnStream.Top;
    btnStop.OnClick := btnStopClick;
  end;

  if not Assigned(cbFFT) then
  begin
    cbFFT := TComboBox.Create(Self);
    cbFFT.Parent := Self;
    cbFFT.Name := 'cbFFT';
    cbFFT.Style := csDropDownList;
    cbFFT.Width := 80;
    cbFFT.Left := edtCenterFreq.Left + edtCenterFreq.Width + 6;
    cbFFT.Top := edtCenterFreq.Top;

    cbFFT.Items.Add('512');
    cbFFT.Items.Add('1024');
    cbFFT.Items.Add('2048');
    cbFFT.Items.Add('4096');

    cbFFT.Text := '1024';
  end;

  if not Assigned(edtFPS) then
  begin
    edtFPS := TEdit.Create(Self);
    edtFPS.Parent := Self;
    edtFPS.Name := 'edtFPS';
    edtFPS.Width := 40;
    edtFPS.Text := '10';
    edtFPS.Left := cbFFT.Left + cbFFT.Width + 6;
    edtFPS.Top := cbFFT.Top;
  end;

  if not Assigned(edtStep) then
  begin
    edtStep := TEdit.Create(Self);
    edtStep.Parent := Self;
    edtStep.Name := 'edtStep';
    edtStep.Width := 50;
    edtStep.Text := '60';
    edtStep.Left := EndScanEdit.Left + EndScanEdit.Width + 6;
    edtStep.Top := EndScanEdit.Top;
  end;

  if not Assigned(chkAGC) then
  begin
    chkAGC := TCheckBox.Create(Self);
    chkAGC.Parent := Self;
    chkAGC.Name := 'chkAGC';
    chkAGC.Caption := 'AGC';
    chkAGC.Checked := True;
    chkAGC.Left := tbGain.Left;
    chkAGC.Top := tbGain.Top + tbGain.Height + 4;
    chkAGC.OnClick := chkAGCClick;
  end;

  if not Assigned(cbWidth) then
  begin
    cbWidth := TComboBox.Create(Self);
    cbWidth.Parent := Self;
    cbWidth.Name := 'cbWidth';
    cbWidth.Style := csDropDownList;
    cbWidth.Width := 55;
    cbWidth.Left := cbSampleRate.Left + cbSampleRate.Width + 6;
    cbWidth.Top := cbSampleRate.Top;

    cbWidth.Items.Add('20');
    cbWidth.Items.Add('40');

    cbWidth.Text := '40';
    cbWidth.ItemIndex := 1;
    cbWidth.OnChange := cbWidthChange;
  end;

  if not Assigned(btnChkRange) then
  begin
    btnChkRange := TButton.Create(Self);
    btnChkRange.Parent := Self;
    btnChkRange.Name := 'btnChkRange';
    btnChkRange.Caption := 'ChkRange';
    btnChkRange.Width := 70;
    btnChkRange.Left := btnFastSweep.Left + btnFastSweep.Width + 6;
    btnChkRange.Top := btnFastSweep.Top;
    btnChkRange.OnClick := btnChkRangeClick;
  end;


if not Assigned(chkPeakHold) then
begin
  chkPeakHold := TCheckBox.Create(Self);
  chkPeakHold.Parent := Self;
  chkPeakHold.Name := 'chkPeakHold';
  chkPeakHold.Caption := 'PeakHold';
  chkPeakHold.Width := 75;
  chkPeakHold.Left := ChkWateFall.Left + ChkWateFall.Width + 8;
  chkPeakHold.Top := ChkWateFall.Top;
  chkPeakHold.Checked := False;
  chkPeakHold.OnClick := chkPeakHoldClick;
end;
end;

function TForm1.GetFFTSize: Integer;
begin
  Result := 1024;

  if Assigned(cbFFT) then
    Result := StrToIntDef(cbFFT.Text, 1024);

  if (Result <> 512) and
     (Result <> 1024) and
     (Result <> 2048) and
     (Result <> 4096) then
    Result := 1024;
end;

function TForm1.GetFPS: Integer;
begin
  Result := 10;

  if Assigned(edtFPS) then
    Result := StrToIntDef(edtFPS.Text, 10);

  if Result < 1 then Result := 1;
  if Result > 30 then Result := 30;
end;

function TForm1.GetStepMHz: Integer;
begin
  Result := 60;

  if Assigned(edtStep) then
    Result := StrToIntDef(edtStep.Text, 60);

  if Result < 1 then Result := 1;
  if Result > 60 then Result := 60;
end;

function TForm1.GetIQDecimation: Integer;
begin
  Result := 1024;

  if Assigned(cbIQDecim) then
    Result := StrToIntDef(cbIQDecim.Text, 1024);

  if (Result <> 256) and
     (Result <> 1024) then
    Result := 1024;
end;

procedure TForm1.UpdateButtons;
var
  Connected: Boolean;
  ModeActive: Boolean;
  ThreadAlive: Boolean;
  ConfigAllowed: Boolean;
begin
  Connected := COM_IsOpen;

  ModeActive :=
    FStreaming or
    FSweeping or
    FIQStreaming;

  ThreadAlive := Assigned(FRxThread);

  ConfigAllowed :=
    Connected and
    (not ModeActive) and
    (not ThreadAlive) and
    (not FChkRunning);

  if Assigned(btnStream) then
  begin
    btnStream.Enabled :=
      Connected and
      (not FSweeping) and
      (not FIQStreaming);

    if FStreaming then
      btnStream.Caption := 'Stop Strm'
    else
      btnStream.Caption := 'Stream';
  end;

  btnFastSweep.Enabled :=
    Connected and
    (not FStreaming) and
    (not FIQStreaming);

  if FSweeping then
    btnFastSweep.Caption := 'Stop Sweep'
  else
    btnFastSweep.Caption := 'Fast Sweep';

  if Assigned(btnIQStream) then
  begin
    btnIQStream.Enabled :=
      Connected and
      (not FStreaming) and
      (not FSweeping);

    if FIQStreaming then
      btnIQStream.Caption := 'Stop IQ'
    else
      btnIQStream.Caption := 'IQ Stream';
  end;

  if Assigned(btnStop) then
    btnStop.Enabled :=
      Connected and
      (ModeActive or ThreadAlive);

  if Assigned(cbIQDecim) then
    cbIQDecim.Enabled := ConfigAllowed;

  if Assigned(btnChkRange) then
  begin
    if FChkRunning then
      btnChkRange.Enabled := True
    else
      btnChkRange.Enabled := ConfigAllowed;
  end;

  cbSampleRate.Enabled := ConfigAllowed;
  tbGain.Enabled := ConfigAllowed;

  if Assigned(chkAGC) then
    chkAGC.Enabled := ConfigAllowed;

  if Assigned(cbWidth) then
    cbWidth.Enabled := ConfigAllowed;

  if Assigned(cbFFT) then
    cbFFT.Enabled := ConfigAllowed;

  if Assigned(edtFPS) then
    edtFPS.Enabled := ConfigAllowed;

  if Assigned(edtStep) then
    edtStep.Enabled := ConfigAllowed;

  StarScanEdit.Enabled := ConfigAllowed;
  EndScanEdit.Enabled := ConfigAllowed;
end;

// =============================================================================
// Commands before receiver thread starts
// =============================================================================

function TForm1.SdrWaitOK(TimeoutMs: Cardinal): Boolean;
var
  T0: DWORD;
  S: AnsiString;
begin
  Result := False;

  T0 := GetTickCount;

  while GetTickCount - T0 < TimeoutMs do
  begin
    if not SdrReadLine(S, 200) then
      Continue;

    S := Trim(S);

    if S = '' then
      Continue;

  //  Log('<< ' + S);

    if S = 'OK' then
    begin
      Result := True;
      Exit;
    end;

    if Copy(S, 1, 3) = 'ERR' then
      Exit;
  end;
end;

function TForm1.WaitIdle(TimeoutMs: Cardinal): Boolean;
var
  T0: DWORD;
  S: AnsiString;
begin
  Result := False;

  { Direct COM read allowed only if no RX thread exists. }
  if Assigned(FRxThread) then
    Exit;

  T0 := GetTickCount;

  while GetTickCount - T0 < TimeoutMs do
  begin
    if not SdrReadLine(S, 200) then
      Continue;

    S := Trim(S);

    if S = '' then
      Continue;

    Log('<< ' + S);

    if S = '#IDLE' then
    begin
      Result := True;
      Exit;
    end;
  end;
end;

function TForm1.SendAndWaitOK(const Cmd: AnsiString;
  TimeoutMs: Cardinal): Boolean;
begin
  Result := False;

  if not COM_IsOpen then
    Exit;

  if Assigned(FRxThread) then
  begin
    Log('Direct command denied: RX thread is active');
    Exit;
  end;

  Log('>> ' + Cmd);

  if not SdrSendLine(Cmd) then
  begin
    Log('COM write failed');
    Exit;
  end;

  Result := SdrWaitOK(TimeoutMs);

  if not Result then
    Log('Command rejected or timeout: ' + Cmd);
end;

function TForm1.ConfigureReceiverFromUI: Boolean;
var
  Width: Integer;
  Rate: Integer;
  Gain: Integer;
begin
  Result := False;

  if Assigned(FRxThread) then
    Exit;

  Width := 40;

  if Assigned(cbWidth) then
    Width := StrToIntDef(Trim(cbWidth.Text), 40);

  if (Width <> 20) and
     (Width <> 40) then
    Width := 40;

  Rate := StrToIntDef(Trim(cbSampleRate.Text), 80);

  if (Rate <> 80) and
     (Rate <> 40) and
     (Rate <> 16) then
    Rate := 80;

  Gain := tbGain.Position;

  if not SendAndWaitOK(
    'WIDTH ' + IntToStr(Width), 1500) then
    Exit;

  if not SendAndWaitOK(
    'RATE ' + IntToStr(Rate), 1500) then
    Exit;

  if Assigned(chkAGC) and chkAGC.Checked then
  begin
    if not SendAndWaitOK('GAIN HARDWARE', 1500) then
      Exit;
  end
  else
  begin
    if not SendAndWaitOK(
      'GAIN MANUAL ' + IntToStr(Gain), 1500) then
      Exit;
  end;

  Result := True;
end;

// =============================================================================
// Modes
// =============================================================================

procedure TForm1.StartReceiverThread;
begin
  if Assigned(FRxThread) then
    Exit;

  FRxThread := TSdrRxThread.Create(Self);
  FRxThread.Resume;
end;

procedure TForm1.StartStream;
var
  Frequency: Integer;
  FFT: Integer;
  FPS: Integer;
  Cmd: AnsiString;
begin
  if not COM_IsOpen then
    Exit;

  if Assigned(FRxThread) then
    StopStreaming;

  if not ConfigureReceiverFromUI then
    Exit;

  Frequency :=
    StrToIntDef(Trim(edtCenterFreq.Text), 2440);

  if Frequency < SDR_MIN_MHZ then
    Frequency := SDR_MIN_MHZ;

  if Frequency > SDR_MAX_MHZ then
    Frequency := SDR_MAX_MHZ;

  edtCenterFreq.Text := IntToStr(Frequency);

  FFT := GetFFTSize;
  FPS := GetFPS;

  Cmd := Format('STREAM %d %d %d',
    [Frequency, FFT, FPS]);

  COM_Purge;

  Log('>> ' + Cmd);

  if not SdrSendLine(Cmd) then
  begin
    Log('STREAM COM write failed');
    Exit;
  end;

  { Receive only initial OK.
    #STREAM_READY and binary frames belong to receiver thread. }
  if not SdrWaitOK(1500) then
  begin
    Log('STREAM rejected');
    COM_Purge;
    Exit;
  end;

  FStreaming := True;
  FSweeping := False;
  FIQStreaming := False;

  FFrames := 0;
  FFpsTick := GetTickCount;

  FSmInit := False;
  FSmInit2 := False;

  ResetWaterfall;

  StartReceiverThread;
  UpdateButtons;
end;





function SweepCalcBins(FirstMHz, LastMHz, StepMHz, FFT: Integer; out Kp, Steps: Integer): Integer;
var
  BinHz, Ash: Double;
  Mk: Integer;
begin
  Result := 0; Kp := 0; Steps := 0;
  BinHz := 80e6 / FFT;
  Kp := Trunc((StepMHz*1e6)/BinHz + 0.5);
  Kp := (Kp + 1) and (not 1);
  Mk := (FFT*3) div 4;
  if Kp > Mk then Kp := Mk;
  if Kp < 16 then Kp := 16;
  Ash := Kp * BinHz;
  Steps := Trunc((LastMHz-FirstMHz)*1e6 / Ash + 0.999999);
  if Steps < 1 then Steps := 1;
  Result := Steps * Kp;
end;





procedure TForm1.StartSweep;
var
  FirstMHz, LastMHz: Integer;
  StepMHz, FFT: Integer;
  Cmd: AnsiString;
  Kp, Steps, TotalBins: Integer;
begin
  if not COM_IsOpen then
    Exit;

  if Assigned(FRxThread) then
    StopStreaming;

  FirstMHz :=
    StrToIntDef(Trim(StarScanEdit.Text), 2400);

  LastMHz :=
    StrToIntDef(Trim(EndScanEdit.Text), 2480);

  if FirstMHz < SDR_MIN_MHZ then
    FirstMHz := SDR_MIN_MHZ;

  if LastMHz > SDR_MAX_MHZ then
    LastMHz := SDR_MAX_MHZ;

  if LastMHz <= FirstMHz then
  begin
    Log('SWEEP: stop must be above start');
    Exit;
  end;

  StarScanEdit.Text := IntToStr(FirstMHz);
  EndScanEdit.Text := IntToStr(LastMHz);

  StepMHz := GetStepMHz;
  FFT := GetFFTSize;







    TotalBins := SweepCalcBins(FirstMHz, LastMHz, StepMHz, FFT, Kp, Steps);
  if TotalBins > 16384 then
  begin
    Log(Format('SWEEP too wide: bins=%d max=16384 kp=%d steps=%d - ужми спан/FFT', [TotalBins, Kp, Steps]));
    Exit;
  end;



  if not SendAndWaitOK('RATE 80', 1500) then
    Exit;

  if not SendAndWaitOK('WIDTH 40', 1500) then
    Exit;

  if not SendAndWaitOK('GAIN HARDWARE', 1500) then
    Exit;

  FConfiguring := True;
  try
    cbSampleRate.Text := '80';
    cbSampleRate.ItemIndex := 0;

    if Assigned(cbWidth) then
    begin
      cbWidth.Text := '40';
      cbWidth.ItemIndex := 1;
    end;

    if Assigned(chkAGC) then
      chkAGC.Checked := True;
  finally
    FConfiguring := False;
  end;

  Cmd := Format('SWEEP %d %d %d %d',
    [FirstMHz, LastMHz, StepMHz, FFT]);

  COM_Purge;

  Log('>> ' + Cmd);

  if not SdrSendLine(Cmd) then
  begin
    Log('SWEEP COM write failed');
    Exit;
  end;

  if not SdrWaitOK(1500) then
  begin
    Log('SWEEP rejected');
    COM_Purge;
    Exit;
  end;

  FSweeping := True;
  FStreaming := False;
  FIQStreaming := False;

  FFrames := 0;
  FFpsTick := GetTickCount;

  FSmInit := False;
  FSmInit2 := False;

  ResetWaterfall;

  StartReceiverThread;
  UpdateButtons;
end;

procedure TForm1.StartIQStream;
var
  Frequency: Integer;
  Decimation: Integer;
  Cmd: AnsiString;
begin
  if not COM_IsOpen then
    Exit;

  if Assigned(FRxThread) then
    StopStreaming;

  if not ConfigureReceiverFromUI then
    Exit;

  Frequency :=
    StrToIntDef(Trim(edtCenterFreq.Text), 2440);

  if Frequency < SDR_MIN_MHZ then
    Frequency := SDR_MIN_MHZ;

  if Frequency > SDR_MAX_MHZ then
    Frequency := SDR_MAX_MHZ;

  edtCenterFreq.Text := IntToStr(Frequency);

  Decimation := GetIQDecimation;

  Cmd := Format('IQSTREAM %d %d',
    [Frequency, Decimation]);

  COM_Purge;

  Log('>> ' + Cmd);

  if not SdrSendLine(Cmd) then
  begin
    Log('IQSTREAM COM write failed');
    Exit;
  end;

  if not SdrWaitOK(1500) then
  begin
    Log('IQSTREAM rejected');
    COM_Purge;
    Exit;
  end;

  FIQStreaming := True;
  FStreaming := False;
  FSweeping := False;

  FIQLogTick := 0;

  StartReceiverThread;
  UpdateButtons;
end;

procedure TForm1.StopStreaming;
begin
  { Important:
    while FRxThread exists, only FRxThread reads COM.
    GUI waits on WinAPI event, not SdrReadLine. }

  if Assigned(FRxThread) then
  begin
    if COM_IsOpen then
    begin
      if FIdleEvent <> 0 then
        Windows.ResetEvent(FIdleEvent);

      SdrSendLine('STOP');

      if FIdleEvent <> 0 then
      begin
        if WaitForSingleObject(FIdleEvent, 3000) <>
           WAIT_OBJECT_0 then
          Log('STOP timeout: #IDLE not received');
      end;
    end;

    FRxThread.Terminate;
    FRxThread.WaitFor;

    FreeAndNil(FRxThread);
  end
  else if COM_IsOpen then
  begin
    SdrSendLine('STOP');
    WaitIdle(1500);
  end;

  FStreaming := False;
  FSweeping := False;
  FIQStreaming := False;

  if COM_IsOpen then
    COM_Purge;

  UpdateButtons;
end;

// =============================================================================
// Events
// =============================================================================

procedure TForm1.btnStreamClick(Sender: TObject);
begin
  if FStreaming then
    StopStreaming
  else
    StartStream;

  UpdateButtons;
end;

procedure TForm1.btnFastSweepClick(Sender: TObject);
begin
  if FSweeping then
    StopStreaming
  else
    StartSweep;

  UpdateButtons;
end;
procedure TForm1.btnIQStreamClick(Sender: TObject);
begin
  if FIQStreaming then
    StopStreaming
  else
    StartIQStream;

  UpdateButtons;
end;


procedure TForm1.btnStopClick(Sender: TObject);
begin
  StopStreaming;
end;

procedure TForm1.cbSampleRateChange(Sender: TObject);
begin
  if FConfiguring or
     (not COM_IsOpen) or
     Assigned(FRxThread) then
    Exit;

  SendAndWaitOK(
    'RATE ' + Trim(cbSampleRate.Text), 1500);
end;

procedure TForm1.cbWidthChange(Sender: TObject);
begin
  if FConfiguring or
     (not COM_IsOpen) or
     Assigned(FRxThread) then
    Exit;

  SendAndWaitOK(
    'WIDTH ' + Trim(cbWidth.Text), 1500);
end;

procedure TForm1.tbGainChange(Sender: TObject);
begin
  if FConfiguring or
     (not COM_IsOpen) or
     Assigned(FRxThread) then
    Exit;

  if Assigned(chkAGC) and chkAGC.Checked then
    Exit;

  if GetTickCount - FLastGainTick < 150 then
    Exit;

  FLastGainTick := GetTickCount;

  SendAndWaitOK(
    'GAIN MANUAL ' + IntToStr(tbGain.Position),
    1500);
end;

procedure TForm1.chkAGCClick(Sender: TObject);
begin
  if FConfiguring or
     (not COM_IsOpen) or
     Assigned(FRxThread) then
    Exit;

  if chkAGC.Checked then
    SendAndWaitOK('GAIN HARDWARE', 1500)
  else
    SendAndWaitOK(
      'GAIN MANUAL ' + IntToStr(tbGain.Position),
      1500);
end;




procedure TForm1.ChkWateFallClick(Sender: TObject);
begin
  if ChkWateFall.Checked then
    ResetWaterfall;

  pbScope.Invalidate;
end;

procedure TForm1.upFreqClick(Sender: TObject);
var
  Frequency: Integer;
begin
  Frequency :=
    StrToIntDef(Trim(edtCenterFreq.Text), 2440) + 1;

  if Frequency > SDR_MAX_MHZ then
    Frequency := SDR_MAX_MHZ;

  edtCenterFreq.Text := IntToStr(Frequency);

  if FStreaming then
    StartStream
  else if FIQStreaming then
    StartIQStream
  else if (not FSweeping) and
          (not Assigned(FRxThread)) and
          COM_IsOpen then
    SendAndWaitOK(
      'FREQ ' + IntToStr(Frequency), 5000);
end;

procedure TForm1.dwnFreqClick(Sender: TObject);
var
  Frequency: Integer;
begin
  Frequency :=
    StrToIntDef(Trim(edtCenterFreq.Text), 2440) - 1;

  if Frequency < SDR_MIN_MHZ then
    Frequency := SDR_MIN_MHZ;

  edtCenterFreq.Text := IntToStr(Frequency);

  if FStreaming then
    StartStream
  else if FIQStreaming then
    StartIQStream
  else if (not FSweeping) and
          (not Assigned(FRxThread)) and
          COM_IsOpen then
    SendAndWaitOK(
      'FREQ ' + IntToStr(Frequency), 5000);
end;

// =============================================================================
// Range check
// =============================================================================

function ParsePllWindowEx(const S: AnsiString;
  out Cap, First, Len: Integer;
  out LoHz: Cardinal; out Mode: Integer): Boolean;
var
  P0, PL, PP, PR: Integer;
  PLo, PLP, PRP: Integer;
begin
  Result := False;

  Cap := 0;
  First := 0;
  Len := 0;
  LoHz := 0;
  Mode := 0;

  P0 := Pos('pll=', S);
  PL := Pos('[', S);
  PP := Pos('+', S);
  PR := Pos(']', S);

  if (P0 = 0) or
     (PL = 0) or
     (PP = 0) or
     (PR = 0) then
    Exit;

  if not ((P0 < PL) and
          (PL < PP) and
          (PP < PR)) then
    Exit;

  Cap :=
    StrToIntDef(
      Trim(Copy(S, P0 + 4, PL - P0 - 4)), -1);

  First :=
    StrToIntDef(
      Copy(S, PL + 1, PP - PL - 1), -1);

  Len :=
    StrToIntDef(
      Copy(S, PP + 1, PR - PP - 1), -1);

  if (Cap < 0) or
     (First < 0) or
     (Len < 0) then
    Exit;

  PLo := Pos('lo=', S);
  PLP := Pos('(', S);
  PRP := Pos(')', S);

  if (PLo > 0) and
     (PLP > PLo) and
     (PRP > PLP) then
  begin
    LoHz :=
      Cardinal(StrToIntDef(
        Trim(Copy(S, PLo + 3, PLP - PLo - 3)), 0));

    Mode :=
      StrToIntDef(
        Copy(S, PLP + 1, PRP - PLP - 1), 0);
  end;

  Result := True;
end;

function SdrQueryStages(out Cap, First, Len, Mode: Integer;
  out LoHz: Cardinal): Boolean;
var
  T0: DWORD;
  Resp: AnsiString;
begin
  Result := False;

  if Assigned(Form1.FRxThread) then
    Exit;

  if not SdrSendLine('STAGES?') then
    Exit;

  T0 := GetTickCount;

  while GetTickCount - T0 < 2000 do
  begin
    if not SdrReadLine(Resp, 400) then
      Continue;

    Resp := Trim(Resp);

    if Resp = '' then
      Continue;

    if Resp[1] = '#' then
      Continue;

    if Pos('pll=', Resp) = 0 then
      Continue;

    Result :=
      ParsePllWindowEx(
        Resp, Cap, First, Len, LoHz, Mode);

    Exit;
  end;
end;

procedure TForm1.btnChkRangeClick(Sender: TObject);
var
  A, B, F: Integer;
  Total, Done: Integer;

  S, ErrMsg: AnsiString;

  Cap, First, Len, Mode: Integer;
  LoHz: Cardinal;

  T0: DWORD;
  OK: Boolean;
begin
  if FChkRunning then
  begin
    FChkCancel := True;
    Exit;
  end;

  if not COM_IsOpen then
  begin
    Log('COM closed');
    Exit;
  end;

  if Assigned(FRxThread) then
    StopStreaming;

  A :=
    StrToIntDef(Trim(StarScanEdit.Text), SDR_MIN_MHZ);

  B :=
    StrToIntDef(Trim(EndScanEdit.Text), SDR_MAX_MHZ);

  if A < SDR_MIN_MHZ then A := SDR_MIN_MHZ;
  if B > SDR_MAX_MHZ then B := SDR_MAX_MHZ;

  if A > B then
  begin
    Log('Range check: Start > Stop');
    Exit;
  end;

  FChkRunning := True;
  FChkCancel := False;

  btnChkRange.Caption := 'Stop Chk';
  UpdateButtons;

  try
    Total := (B - A) div 100 + 1;
    Done := 0;
    F := A;

    while F <= B do
    begin
      if FChkCancel then
      begin
        Log('<< RANGE CHECK CANCELLED');
        Break;
      end;

      COM_Purge;

      SdrSendLine('FREQ ' + IntToStr(F));

      OK := False;
      ErrMsg := '';
      T0 := GetTickCount;

      while GetTickCount - T0 < 8000 do
      begin
        if not SdrReadLine(S, 500) then
          Continue;

        S := Trim(S);

        if S = '' then
          Continue;

        if S[1] = '#' then
          Continue;

        if S = 'OK' then
        begin
          OK := True;
          Break;
        end;

        if Copy(S, 1, 3) = 'ERR' then
        begin
          ErrMsg := S;
          Break;
        end;
      end;

      if OK then
      begin
        if SdrQueryStages(
          Cap, First, Len, Mode, LoHz) then
          Log(Format(
            'Frequency %d MHz : OK pll=%d[%d+%d] mode=%d',
            [F, Cap, First, Len, Mode]))
        else
          Log(Format(
            'Frequency %d MHz : OK (no stage report)',
            [F]));
      end
      else if ErrMsg <> '' then
        Log(Format(
          'Frequency %d MHz : Failed (%s)',
          [F, ErrMsg]))
      else
        Log(Format(
          'Frequency %d MHz : Failed (timeout)',
          [F]));

      Inc(Done);

      Gauge1.Progress :=
        Round(Done * 100.0 / Total);

      Application.ProcessMessages;

      Inc(F, 100);
    end;
  finally
    FChkRunning := False;
    btnChkRange.Caption := 'ChkRange';
    UpdateButtons;
  end;
end;

// =============================================================================
// Connect / form
// =============================================================================

// =========================== ComPort Helpers =================================

function WaitDeviceOnline(TimeoutMs: Cardinal): Boolean;
var
  TotalStart, ProbeStart: DWORD;
  S: AnsiString;
begin
  Result := False;
  TotalStart := GetTickCount;

  while DWORD(GetTickCount - TotalStart) < TimeoutMs do
  begin
    { ESP мог ещё загружаться. Команда останется во входном
      буфере либо будет повторена на следующей итерации. }
    if not SdrSendLine('PING') then
      Exit;

    ProbeStart := GetTickCount;

    { Ждём ответ на одну попытку около секунды. }
    while (DWORD(GetTickCount - ProbeStart) < 1000) and
          (DWORD(GetTickCount - TotalStart) < TimeoutMs) do
    begin
      if not SdrReadLine(S, 200) then
        Continue;

      S := Trim(S);

      if S = '' then
        Continue;

      Form1.Log('<< ' + string(S));

      if S = 'PONG' then
      begin
        Result := True;
        Exit;
      end;

      { READY можно увидеть при первой загрузке, но само по себе
        оно не обязательно: окончательно готовность подтверждает PONG. }
    end;

    Sleep(100);
  end;
end;

function WaitForExactLine(const Expected: AnsiString;
  TimeoutMs: Cardinal): Boolean;
var
  T0: DWORD;
  S: AnsiString;
begin
  Result := False;
  T0 := GetTickCount;

  while GetTickCount - T0 < TimeoutMs do
  begin
    if not SdrReadLine(S, 300) then
      Continue;

    S := Trim(S);

    if S = '' then
      Continue;

    Form1.Log('<< ' + string(S));

    if S = Expected then
    begin
      Result := True;
      Exit;
    end;
  end;
end;

function WaitForPrefixLine(const Prefix: AnsiString;
  TimeoutMs: Cardinal; out Response: AnsiString): Boolean;
var
  T0: DWORD;
  S: AnsiString;
begin
  Result := False;
  Response := '';
  T0 := GetTickCount;

  while GetTickCount - T0 < TimeoutMs do
  begin
    if not SdrReadLine(S, 300) then
      Continue;

    S := Trim(S);

    if S = '' then
      Continue;

    Form1.Log('<< ' + string(S));

    if Copy(S, 1, Length(Prefix)) = Prefix then
    begin
      Response := S;
      Result := True;
      Exit;
    end;
  end;
end;


procedure TForm1.btnComportConnectDisconnectClick(Sender: TObject);
var
  S: AnsiString;
  Tries: Integer;
  OK: Boolean;
begin
  if not COM_IsOpen then
  begin
    OK := False;

    for Tries := 1 to 5 do
    begin
      if SdrOpen(Trim(edtDeviceCOM.Text)) then
      begin
        OK := True;
        Break;
      end;

      Log(Format('COM open try %d failed', [Tries]));
      Sleep(600);
    end;

    if not OK then
    begin
      Log('COM open failed');
      Exit;
    end;

    btnComportConnectDisconnect.Caption := 'Disconnect';

    Log('COM open ' + edtDeviceCOM.Text);
Log('Waiting for device startup...');

{ Новый скетч загружается дольше из-за PHY, PLL и проверки памяти. }
if not WaitDeviceOnLine(20000) then
begin
  Log('READY not received; checking already running device...');
end;

{ Удаляем только оставшиеся загрузочные сообщения. }
COM_Purge;

if not SdrSendLine('PING') then
begin
  Log('PING write failed');
  SdrClose;
  btnComportConnectDisconnect.Caption := 'Connect';
  UpdateButtons;
  Exit;
end;

if not WaitForExactLine('PONG', 5000) then
begin
  Log('Device does not answer PING');
  SdrClose;
  btnComportConnectDisconnect.Caption := 'Connect';
  UpdateButtons;
  Exit;
end;

Log('Device is ready.');


if SdrSendLine('INFO') then
begin

    if not WaitForPrefixLine('ESPSDR ', 3000, S) then
    Log('INFO response timeout');
end;


  end
  else
  begin
    StopStreaming;
    SdrClose;

    btnComportConnectDisconnect.Caption := 'Connect';
    Log('COM closed');
  end;

  UpdateButtons;
end;

procedure TForm1.FormCreate(Sender: TObject);
begin
  Set8087CW($133F);   { замаскировать исключения FPU }
  
  pbScope.ControlStyle :=
    pbScope.ControlStyle + [csOpaque];

  DoubleBuffered := True;
  OnShow := FormShow;

  FCS := TCriticalSection.Create;

  { Native WinAPI manual-reset event.
    This replaces incompatible Delphi 7 TEvent.Create call. }
  FIdleEvent := Windows.CreateEvent(nil, True, False, nil);

  if FIdleEvent = 0 then
    raise Exception.CreateFmt(
      'CreateEvent failed. Windows error=%d',
      [GetLastError]);

  FBack := TBitmap.Create;
  FBack.PixelFormat := pf24bit;

  FWater := TBitmap.Create;
  FWater.PixelFormat := pf24bit;

  FStreaming := False;
  FSweeping := False;
  FIQStreaming := False;
  FConfiguring := False;

  FSmInit := False;
  FSmInit2 := False;

  FIQNotifyPending := 0;
  FIQLogTick := 0;

  Position := poDesktopCenter;

  if SizeOf(TSpectrumHeader) <> 36 then
    ShowMessage(Format(
      'Spectrum header size mismatch: %d (expected 36)',
      [SizeOf(TSpectrumHeader)]));

  BuildLUT;
  BuildGradientLUT;


FPeakHoldMs := 700;
FPeakDecayDbSec := 4.0;
FPeakUpdateTick := GetTickCount;

  { Confirmed working waterfall settings. }
//  FWaterFloorDb := -100.0;
//  FWaterSpanDb := 90.0;



  FWaterFloorDb := -85.0;
FWaterSpanDb  := 95.0;

  FSmStartHz := 0;
  FSmStopHz := 0;
  SetLength(FSmSpec, 0);

  if edtDeviceCOM.Text = '' then
    edtDeviceCOM.Text := 'COM207';

  if edtCenterFreq.Text = '' then
    edtCenterFreq.Text := '2440';

  if StarScanEdit.Text = '' then
    StarScanEdit.Text := IntToStr(SDR_MIN_MHZ);

  if EndScanEdit.Text = '' then
    EndScanEdit.Text := IntToStr(SDR_MAX_MHZ);

  if cbSampleRate.Items.Count = 0 then
  begin
    cbSampleRate.Items.Add('80');
    cbSampleRate.Items.Add('40');
    cbSampleRate.Items.Add('16');
  end;

  if cbSampleRate.Text = '' then
    cbSampleRate.Text := '80';

  tbGain.Min := 0;
  tbGain.Max := 82;
  tbGain.Frequency := 10;
  tbGain.Position := 60;

  Gauge1.MinValue := 0;
  Gauge1.MaxValue := 100;



  cbIQDecim.Items.Add('1024');
  cbIQDecim.Items.Add('256');

  cbIQDecim.Text := '1024';
  cbIQDecim.ItemIndex := 0;



  EnsureExtraControls;



  FPeakHoldOn := False;
FPeakHoldInit := False;
SetLength(FPeakHoldDb, 0);



  chkAGC.Checked := False;

  if cbSampleRate.ItemIndex < 0 then
    cbSampleRate.ItemIndex := 0;

  if Assigned(cbFFT) and
     (cbFFT.ItemIndex < 0) then
    cbFFT.ItemIndex := 1;


  cbWidth.Items.Add('20');
  cbWidth.Items.Add('40');

  if Assigned(cbWidth) and
     (cbWidth.ItemIndex < 0) then
    cbWidth.ItemIndex := 1;

  cbSampleRate.OnChange := cbSampleRateChange;

  if Assigned(cbWidth) then
    cbWidth.OnChange := cbWidthChange;

  chkAGC.OnClick := chkAGCClick;
  tbGain.OnChange := tbGainChange;

  ChkWateFall.OnClick := ChkWateFallClick;
  pbScope.OnPaint := pbScopePaint;

  FFpsTick := GetTickCount;

  Log('Ready. Open COM, then Stream / Sweep / IQ Stream.');

  UpdateButtons;
end;

procedure TForm1.FormDestroy(Sender: TObject);
begin
  { Receiver thread must finish before FCS / event are destroyed. }
  if Assigned(FRxThread) then
  begin
    if COM_IsOpen then
    begin
      if FIdleEvent <> 0 then
        Windows.ResetEvent(FIdleEvent);

      SdrSendLine('STOP');

      if FIdleEvent <> 0 then
        WaitForSingleObject(FIdleEvent, 500);
    end;

    FRxThread.Terminate;
    FRxThread.WaitFor;

    FreeAndNil(FRxThread);
  end;

  if COM_IsOpen then
    SdrClose;

  FreeAndNil(FBack);
  FreeAndNil(FWater);

  if FIdleEvent <> 0 then
  begin
    CloseHandle(FIdleEvent);
    FIdleEvent := 0;
  end;

  FreeAndNil(FCS);
end;

procedure TForm1.FormShow(Sender: TObject);
begin
  if (cbSampleRate.Items.Count > 0) and
     (cbSampleRate.ItemIndex < 0) then
    cbSampleRate.ItemIndex := 0;

  if Assigned(cbFFT) and
     (cbFFT.ItemIndex < 0) then
    cbFFT.ItemIndex := 1;
end;

// =============================================================================
// Scope paint
// =============================================================================

procedure TForm1.pbScopePaint(Sender: TObject);
var
  W, H, SpecH: Integer;
  I, X, Y, N, C0, C1, J: Integer;

  MinDb, MaxDb, SumDb, Db, AvgDb: Single;
  TMin, TMax: Single;
  DisplayMin, DisplayMax, RangeDb: Single;

  BaseY, Cnt: Integer;
  F: Double;

  LinePts: array of TPoint;
  Row: PByteArray;

  YTop, Span, TIndex, A, Inv, O: Integer;

  FrameStart, FrameStop: Double;
  ViewStart, ViewStop: Double;
  BinHz, Fx0, Fx1, FMid, PosBin, Frac: Double;

  I0: Integer;

  HoldPts: array of TPoint;
  HoldDb: Single;

  function DbToY(Value: Single): Integer;
  begin
    if RangeDb <= 0 then
      Result := SpecH - 4
    else
      Result :=
        SpecH - 4 -
        Round((Value - DisplayMin) / RangeDb *
          (SpecH - 28));

    if Result < 18 then Result := 18;
    if Result > SpecH - 4 then Result := SpecH - 4;
  end;

begin
  W := pbScope.Width;
  H := pbScope.Height;

  if (W < 10) or
     (H < 10) then
    Exit;

  if (FBack.Width <> W) or
     (FBack.Height <> H) then
  begin
    FBack.Width := W;
    FBack.Height := H;
    FBack.PixelFormat := pf24bit;
  end;

  FBack.Canvas.Brush.Color := clBlack;
  FBack.Canvas.Brush.Style := bsSolid;
  FBack.Canvas.FillRect(Rect(0, 0, W, H));

  if ChkWateFall.Checked then
  begin
    SpecH := H * 60 div 100;
    if SpecH < 80 then SpecH := 80;
  end
  else
    SpecH := H;

  N := Length(FSmSpec);

  if (N < 16) or
     (not FSmInit2) then
  begin
    FBack.Canvas.Font.Color := clYellow;
    FBack.Canvas.Brush.Style := bsClear;

    FBack.Canvas.TextOut(
      10, 10,
      'Waiting for spectrum... Open COM, Stream/Sweep');

    pbScope.Canvas.Draw(0, 0, FBack);
    Exit;
  end;

  FrameStart := FSmStartHz;
  FrameStop := FSmStopHz;

  if FrameStop <= FrameStart then
  begin
    FrameStart := FDrawHdr.StartHz;
    FrameStop := FDrawHdr.StopHz;
  end;

  if FrameStop <= FrameStart then
    Exit;

  ViewStart := FrameStart;
  ViewStop := FrameStop;

  BinHz := (FrameStop - FrameStart) / N;

  if BinHz <= 0 then
    Exit;

  MinDb := 1e30;
  MaxDb := -1e30;
  SumDb := 0;

  for I := 0 to N - 1 do
  begin
    Db := FSmSpec[I];

    if Db < MinDb then MinDb := Db;
    if Db > MaxDb then MaxDb := Db;

    SumDb := SumDb + Db;
  end;

  Cnt := N;
  AvgDb := SumDb / Cnt;

  TMin := AvgDb - 8;
  TMax := AvgDb + 30;

  if MinDb < TMin then TMin := MinDb - 3;
  if MaxDb > TMax then TMax := MaxDb + 3;

  if TMax - TMin < 30 then
    TMin := TMax - 30;

  if not FSmInit then
  begin
    FSmMin := TMin;
    FSmMax := TMax;
    FSmInit := True;
  end
  else
  begin
    if TMax > FSmMax then
      FSmMax := TMax
    else
      FSmMax := FSmMax * 0.9 + TMax * 0.1;

    if TMin < FSmMin then
      FSmMin := TMin
    else
      FSmMin := FSmMin * 0.9 + TMin * 0.1;
  end;

  DisplayMin := FSmMin;
  DisplayMax := FSmMax;

  RangeDb := DisplayMax - DisplayMin;

  if RangeDb <= 0 then
    RangeDb := 1;

  BaseY := SpecH - 4;

  FBack.Canvas.Pen.Style := psSolid;
  FBack.Canvas.Pen.Width := 1;
  FBack.Canvas.Pen.Color := $00333333;

  for I := 1 to 5 do
  begin
    Y := 18 + ((SpecH - 28) * I) div 6;
    FBack.Canvas.MoveTo(0, Y);
    FBack.Canvas.LineTo(W, Y);
  end;

  FBack.Canvas.Pen.Color := $00404040;














  for I := 1 to 9 do
  begin
    X := (W * I) div 10;
    FBack.Canvas.MoveTo(X, 0);
    FBack.Canvas.LineTo(X, SpecH);
  end;

  if Length(FYTop) <> W then
    SetLength(FYTop, W);

  for X := 0 to W - 1 do
  begin
    Fx0 :=
      ViewStart +
      (ViewStop - ViewStart) * X / W;

    Fx1 :=
      ViewStart +
      (ViewStop - ViewStart) * (X + 1) / W;

    if (Fx1 <= FrameStart) or
       (Fx0 >= FrameStop) then
    begin
      FYTop[X] := BaseY;
      Continue;
    end;

    if Fx0 < FrameStart then Fx0 := FrameStart;
    if Fx1 > FrameStop then Fx1 := FrameStop;

    C0 := Trunc((Fx0 - FrameStart) / BinHz);
    C1 := Trunc((Fx1 - FrameStart - 0.001) / BinHz);

    if C0 < 0 then C0 := 0;
    if C1 >= N then C1 := N - 1;

    if C1 > C0 then
    begin
      Db := -200;

      for J := C0 to C1 do
        if FSmSpec[J] > Db then
          Db := FSmSpec[J];
    end
    else
    begin
      FMid := (Fx0 + Fx1) * 0.5;
      PosBin := (FMid - FrameStart) / BinHz - 0.5;

      if PosBin < 0 then PosBin := 0;
      if PosBin > N - 1 then PosBin := N - 1;

      I0 := Trunc(PosBin);
      Frac := PosBin - I0;

      if I0 >= N - 1 then
        Db := FSmSpec[N - 1]
      else
        Db :=
          FSmSpec[I0] * (1 - Frac) +
          FSmSpec[I0 + 1] * Frac;
    end;

    FYTop[X] := DbToY(Db);
  end;

  for Y := 18 to BaseY do
  begin
    Row := FBack.ScanLine[Y];

    for X := 0 to W - 1 do
    begin
      YTop := FYTop[X];

      if Y < YTop then
        Continue;

      Span := BaseY - YTop;
      if Span < 1 then Span := 1;

      TIndex := ((BaseY - Y) * 255) div Span;

      if TIndex < 0 then TIndex := 0;
      if TIndex > 255 then TIndex := 255;

      A := FGradA[TIndex];
      Inv := 255 - A;

      O := X * 3;

      Row^[O] :=
        (Row^[O] * Inv +
         FGradB[TIndex] * A + 128) shr 8;

      Row^[O + 1] :=
        (Row^[O + 1] * Inv +
         FGradG[TIndex] * A + 128) shr 8;

      Row^[O + 2] :=
        (Row^[O + 2] * Inv +
         FGradR[TIndex] * A + 128) shr 8;
    end;
  end;

  SetLength(LinePts, W);

  for X := 0 to W - 1 do
    LinePts[X] := Point(X, FYTop[X]);

  FBack.Canvas.Pen.Style := psSolid;
  FBack.Canvas.Pen.Width := 3;
  FBack.Canvas.Pen.Color := RGB(0, 85, 0);
  FBack.Canvas.Polyline(LinePts);

  FBack.Canvas.Pen.Width := 1;
  FBack.Canvas.Pen.Color := RGB(140, 255, 140);
  FBack.Canvas.Polyline(LinePts);





  FBack.Canvas.Pen.Width := 1;
  FBack.Canvas.Pen.Color := RGB(140, 255, 140);
  FBack.Canvas.Polyline(LinePts);







  // --- Peak Hold поверх живого ---
if FPeakHoldOn and FPeakHoldInit and (Length(FPeakHoldDb) = N) then
begin
  SetLength(HoldPts, W);
  for X := 0 to W - 1 do
  begin
    Fx0 := ViewStart + (ViewStop - ViewStart) * X / W;
    Fx1 := ViewStart + (ViewStop - ViewStart) * (X + 1) / W;
    if (Fx1 <= FrameStart) or (Fx0 >= FrameStop) then
    begin HoldPts[X] := Point(X, BaseY); Continue; end;
    if Fx0 < FrameStart then Fx0 := FrameStart;
    if Fx1 > FrameStop then Fx1 := FrameStop;
    C0 := Trunc((Fx0 - FrameStart) / BinHz);
    C1 := Trunc((Fx1 - FrameStart - 0.001) / BinHz);
    if C0 < 0 then C0 := 0;
    if C1 >= N then C1 := N - 1;
    if C1 > C0 then
    begin
      HoldDb := -200;
      for J := C0 to C1 do
        if FPeakHoldDb[J] > HoldDb then HoldDb := FPeakHoldDb[J];
    end
    else
    begin
      FMid := (Fx0 + Fx1) * 0.5;
      PosBin := (FMid - FrameStart) / BinHz - 0.5;
      if PosBin < 0 then PosBin := 0;
      if PosBin > N - 1 then PosBin := N - 1;
      I0 := Trunc(PosBin); Frac := PosBin - I0;
      if I0 >= N - 1 then HoldDb := FPeakHoldDb[N - 1]
      else HoldDb := FPeakHoldDb[I0] * (1 - Frac) + FPeakHoldDb[I0 + 1] * Frac;
    end;
    HoldPts[X] := Point(X, DbToY(HoldDb));
  end;
  FBack.Canvas.Pen.Style := psSolid;
  FBack.Canvas.Pen.Width := 1;
  FBack.Canvas.Pen.Color := RGB(255, 180, 60);
  FBack.Canvas.Polyline(HoldPts);

  FBack.Canvas.Brush.Style := bsClear;
  FBack.Canvas.Font.Color := RGB(255, 180, 60);
  FBack.Canvas.Font.Size := 8;
  FBack.Canvas.TextOut(10, 30, 'PEAK HOLD');
end;



  FBack.Canvas.Brush.Style := bsClear;
  FBack.Canvas.Font.Color := clSkyBlue;
  FBack.Canvas.Font.Size := 8;

  for I := 0 to 5 do
  begin
    Db := DisplayMax - RangeDb * I / 5;

    FBack.Canvas.TextOut(
      2,
      DbToY(Db) - 7,
      Format('%.0f', [Db]));
  end;

  FBack.Canvas.Font.Color := clYellow;

  for I := 1 to 9 do
  begin
    F :=
      ViewStart +
      (ViewStop - ViewStart) * I / 10;

    X := (I * W) div 10 - 20;

    if X < 2 then
      X := 2;

    FBack.Canvas.TextOut(
      X, 4,
      Format('%.0f', [F / 1e6]));
  end;

  if ChkWateFall.Checked and
     Assigned(FWater) and
     (FWater.Width > 0) then
  begin
    FBack.Canvas.StretchDraw(
      Rect(0, SpecH + 2, W, H),
      FWater);
  end;

  pbScope.Canvas.Draw(0, 0, FBack);
end;

end.
