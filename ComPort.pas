unit ComPort;   // DO NOT EDIT

interface

uses
  Windows, SysUtils;

const
  DTR_CONTROL_DISABLE   = $00;
  DTR_CONTROL_ENABLE    = $01;
  DTR_CONTROL_HANDSHAKE = $02;

  RTS_CONTROL_DISABLE   = $00;
  RTS_CONTROL_ENABLE    = $01;
  RTS_CONTROL_HANDSHAKE = $02;
  RTS_CONTROL_TOGGLE    = $03;

type
  TParity   = Byte;
  TStopBits = Byte;

function COM_IsOpen: Boolean;

function COM_WriteByte(B: Byte): Boolean;
function COM_ReadByte(var B: Byte): Boolean;

function COM_Read(var Buf; Count: DWORD; out ReadCnt: DWORD): Boolean;
function COM_Write(const Buf; Count: DWORD; out Written: DWORD): Boolean;

function COM_WaitingData: DWORD;
function COM_SendingData: DWORD;

function ApplyDCB(Baud: DWORD; ByteSize: Byte;
  Parity: TParity; StopBits: TStopBits): Boolean;

procedure COM_SetRTS(Enable: Boolean);
procedure COM_SetDTR(Enable: Boolean);

procedure COM_Close;
procedure COM_Purge;
procedure COM_Flush;

function COM_Open(const Port: string; Baud: DWORD;
  ByteSize: Byte; Parity: TParity; StopBits: TStopBits): Boolean;

function COM_SetBaud(Baud: DWORD): Boolean;
function COM_SetLine(ByteSize: Byte; Parity: TParity;
  StopBits: TStopBits): Boolean;

function COM_SetTimeouts(ReadInterval, ReadConst, WriteConst,
  ReadMult, WriteMult: DWORD): Boolean;

function COM_SetBuffers(RecvSize, SendSize: DWORD): Boolean;

function COM_GetModemStatus: DWORD;
function COM_GetCTS: Boolean;
function COM_GetDSR: Boolean;
function COM_GetCarrier: Boolean;
function COM_GetRing: Boolean;

procedure COM_SetBreak(DurationMs: Cardinal);
procedure COM_ResetCard;

const
  COM_CHUNK = 16384;

procedure SendString(const Data: AnsiString);
procedure SendBlock(const Data: AnsiString);

function Send2Com(const Data: AnsiString): Boolean;
function SendByte(const B: Byte): Boolean;

function ReadCom(var Data: AnsiString;
  TimeoutMs: Cardinal = 100): Boolean;

function SendBigBlock(const Block: AnsiString): Boolean;

function ReadBigBlock(var Block: AnsiString; Size: Integer;
  TimeoutMs: Cardinal = 5000): Boolean;

implementation

var
  hSerial: THandle = INVALID_HANDLE_VALUE;
  FSerialLock: TRTLCriticalSection;

function COM_Path(const Port: string): string;
begin
  if Pos('\\.\', Port) = 1 then
    Result := Port
  else
    Result := '\\.\' + Port;
end;

function COM_IsOpen: Boolean;
begin
  EnterCriticalSection(FSerialLock);
  try
    Result := hSerial <> INVALID_HANDLE_VALUE;
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

procedure EnsureBinaryFlag(var DCBRec: TDCB);
begin
  DCBRec.Flags := DCBRec.Flags or 1;
end;

function ApplyDCB(Baud: DWORD; ByteSize: Byte;
  Parity: TParity; StopBits: TStopBits): Boolean;
var
  DCBRec: TDCB;
begin
  EnterCriticalSection(FSerialLock);
  try
    Result := False;

    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    FillChar(DCBRec, SizeOf(DCBRec), 0);
    DCBRec.DCBlength := SizeOf(DCBRec);

    if not GetCommState(hSerial, DCBRec) then
      Exit;

    DCBRec.BaudRate := Baud;
    DCBRec.ByteSize := ByteSize;
    DCBRec.Parity := Parity;
    DCBRec.StopBits := StopBits;

    EnsureBinaryFlag(DCBRec);

    Result := SetCommState(hSerial, DCBRec);

    if Result then
      Sleep(2);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

function COM_Open(const Port: string; Baud: DWORD;
  ByteSize: Byte; Parity: TParity; StopBits: TStopBits): Boolean;
var
  Path: string;
begin
  EnterCriticalSection(FSerialLock);
  try
    Result := False;

    if hSerial <> INVALID_HANDLE_VALUE then
      COM_Close;

    Path := COM_Path(Port);

    hSerial := CreateFile(
      PChar(Path),
      GENERIC_READ or GENERIC_WRITE,
      0,
      nil,
      OPEN_EXISTING,
      FILE_ATTRIBUTE_NORMAL,
      0);

    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    if not ApplyDCB(Baud, ByteSize, Parity, StopBits) then
    begin
      COM_Close;
      Exit;
    end;

    Result := True;
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

procedure COM_Close;
begin
  EnterCriticalSection(FSerialLock);
  try
    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    EscapeCommFunction(hSerial, CLRRTS);
    EscapeCommFunction(hSerial, CLRDTR);

    PurgeComm(hSerial,
      PURGE_RXABORT or PURGE_RXCLEAR or
      PURGE_TXABORT or PURGE_TXCLEAR);

    CloseHandle(hSerial);
    hSerial := INVALID_HANDLE_VALUE;
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

function COM_SetBaud(Baud: DWORD): Boolean;
var
  DCBRec: TDCB;
begin
  EnterCriticalSection(FSerialLock);
  try
    Result := False;

    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    FillChar(DCBRec, SizeOf(DCBRec), 0);
    DCBRec.DCBlength := SizeOf(DCBRec);

    if not GetCommState(hSerial, DCBRec) then
      Exit;

    DCBRec.BaudRate := Baud;
    EnsureBinaryFlag(DCBRec);

    Result := SetCommState(hSerial, DCBRec);

    if Result then
      Sleep(2);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

function COM_SetLine(ByteSize: Byte; Parity: TParity;
  StopBits: TStopBits): Boolean;
var
  DCBRec: TDCB;
begin
  EnterCriticalSection(FSerialLock);
  try
    Result := False;

    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    FillChar(DCBRec, SizeOf(DCBRec), 0);
    DCBRec.DCBlength := SizeOf(DCBRec);

    if not GetCommState(hSerial, DCBRec) then
      Exit;

    DCBRec.ByteSize := ByteSize;
    DCBRec.Parity := Parity;
    DCBRec.StopBits := StopBits;

    EnsureBinaryFlag(DCBRec);

    Result := SetCommState(hSerial, DCBRec);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

function COM_SetTimeouts(ReadInterval, ReadConst, WriteConst,
  ReadMult, WriteMult: DWORD): Boolean;
var
  TOs: TCommTimeouts;
begin
  EnterCriticalSection(FSerialLock);
  try
    Result := False;

    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    FillChar(TOs, SizeOf(TOs), 0);

    TOs.ReadIntervalTimeout := ReadInterval;
    TOs.ReadTotalTimeoutMultiplier := ReadMult;
    TOs.ReadTotalTimeoutConstant := ReadConst;

    TOs.WriteTotalTimeoutMultiplier := WriteMult;
    TOs.WriteTotalTimeoutConstant := WriteConst;

    Result := SetCommTimeouts(hSerial, TOs);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

function COM_SetBuffers(RecvSize, SendSize: DWORD): Boolean;
begin
  EnterCriticalSection(FSerialLock);
  try
    Result := False;

    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    Result := SetupComm(hSerial, RecvSize, SendSize);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

procedure COM_Purge;
begin
  EnterCriticalSection(FSerialLock);
  try
    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    PurgeComm(hSerial,
      PURGE_RXABORT or PURGE_RXCLEAR or
      PURGE_TXABORT or PURGE_TXCLEAR);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

procedure COM_Flush;
begin
  EnterCriticalSection(FSerialLock);
  try
    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    FlushFileBuffers(hSerial);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

function COM_Write(const Buf; Count: DWORD; out Written: DWORD): Boolean;
begin
  Written := 0;

  EnterCriticalSection(FSerialLock);
  try
    Result :=
      (hSerial <> INVALID_HANDLE_VALUE) and
      WriteFile(hSerial, Buf, Count, Written, nil);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

function COM_Read(var Buf; Count: DWORD; out ReadCnt: DWORD): Boolean;
begin
  ReadCnt := 0;

  EnterCriticalSection(FSerialLock);
  try
    Result :=
      (hSerial <> INVALID_HANDLE_VALUE) and
      ReadFile(hSerial, Buf, Count, ReadCnt, nil);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

function COM_WriteByte(B: Byte): Boolean;
var
  Written: DWORD;
begin
  Result := COM_Write(B, 1, Written) and (Written = 1);
end;

function COM_ReadByte(var B: Byte): Boolean;
var
  ReadCnt: DWORD;
begin
  Result := COM_Read(B, 1, ReadCnt) and (ReadCnt = 1);
end;

function COM_WaitingData: DWORD;
var
  Stat: TComStat;
  Err: DWORD;
begin
  Result := 0;

  EnterCriticalSection(FSerialLock);
  try
    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    FillChar(Stat, SizeOf(Stat), 0);
    Err := 0;

    if ClearCommError(hSerial, Err, @Stat) then
      Result := Stat.cbInQue;
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

function COM_SendingData: DWORD;
var
  Stat: TComStat;
  Err: DWORD;
begin
  Result := 0;

  EnterCriticalSection(FSerialLock);
  try
    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    FillChar(Stat, SizeOf(Stat), 0);
    Err := 0;

    if ClearCommError(hSerial, Err, @Stat) then
      Result := Stat.cbOutQue;
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

procedure COM_SetRTS(Enable: Boolean);
begin
  EnterCriticalSection(FSerialLock);
  try
    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    if Enable then
      EscapeCommFunction(hSerial, SETRTS)
    else
      EscapeCommFunction(hSerial, CLRRTS);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

procedure COM_SetDTR(Enable: Boolean);
begin
  EnterCriticalSection(FSerialLock);
  try
    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    if Enable then
      EscapeCommFunction(hSerial, SETDTR)
    else
      EscapeCommFunction(hSerial, CLRDTR);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

procedure COM_ResetCard;
begin
  EnterCriticalSection(FSerialLock);
  try
    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    EscapeCommFunction(hSerial, CLRRTS);
    EscapeCommFunction(hSerial, CLRDTR);

    Sleep(50);

    EscapeCommFunction(hSerial, SETRTS);
    EscapeCommFunction(hSerial, SETDTR);

    Sleep(100);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

function COM_GetModemStatus: DWORD;
begin
  Result := 0;

  EnterCriticalSection(FSerialLock);
  try
    if hSerial <> INVALID_HANDLE_VALUE then
      GetCommModemStatus(hSerial, Result);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

function COM_GetCTS: Boolean;
begin
  Result := (COM_GetModemStatus and MS_CTS_ON) <> 0;
end;

function COM_GetDSR: Boolean;
begin
  Result := (COM_GetModemStatus and MS_DSR_ON) <> 0;
end;

function COM_GetCarrier: Boolean;
begin
  Result := (COM_GetModemStatus and MS_RLSD_ON) <> 0;
end;

function COM_GetRing: Boolean;
begin
  Result := (COM_GetModemStatus and MS_RING_ON) <> 0;
end;

procedure COM_SetBreak(DurationMs: Cardinal);
begin
  EnterCriticalSection(FSerialLock);
  try
    if hSerial = INVALID_HANDLE_VALUE then
      Exit;

    SetCommBreak(hSerial);
    Sleep(DurationMs);
    ClearCommBreak(hSerial);
  finally
    LeaveCriticalSection(FSerialLock);
  end;
end;

procedure SendString(const Data: AnsiString);
var
  Written: DWORD;
begin
  if Length(Data) = 0 then
    Exit;

  COM_Write(Data[1], Length(Data), Written);
end;

procedure SendBlock(const Data: AnsiString);
var
  L: Integer;
  Written: DWORD;
begin
  if not COM_IsOpen then
    Exit;

  L := Length(Data);

  if not COM_Write(L, SizeOf(L), Written) or
     (Written <> SizeOf(L)) then
    Exit;

  if L > 0 then
    COM_Write(Data[1], L, Written);
end;

function Send2Com(const Data: AnsiString): Boolean;
var
  Written: DWORD;
begin
  Result := False;

  if Length(Data) = 0 then
  begin
    Result := True;
    Exit;
  end;

  Result :=
    COM_Write(Data[1], Length(Data), Written) and
    (Written = DWORD(Length(Data)));
end;

function SendByte(const B: Byte): Boolean;
begin
  Result := COM_WriteByte(B);
end;

function ReadCom(var Data: AnsiString;
  TimeoutMs: Cardinal): Boolean;
var
  Buf: array[0..COM_CHUNK - 1] of Byte;
  Need, Got: DWORD;
  T0: DWORD;
  OldLen: Integer;
begin
  Data := '';
  Result := False;

  if not COM_IsOpen then
    Exit;

  T0 := GetTickCount;

  while GetTickCount - T0 < TimeoutMs do
  begin
    Need := COM_WaitingData;

    if Need = 0 then
    begin
      Sleep(2);
      Continue;
    end;

    if Need > COM_CHUNK then
      Need := COM_CHUNK;

    if not COM_Read(Buf[0], Need, Got) then
      Exit;

    if Got = 0 then
    begin
      Sleep(1);
      Continue;
    end;

    OldLen := Length(Data);
    SetLength(Data, OldLen + Integer(Got));

    Move(Buf[0], Data[OldLen + 1], Got);

    Result := True;
    Exit;
  end;
end;

function SendBigBlock(const Block: AnsiString): Boolean;
var
  Total, Sent, Chunk: DWORD;
begin
  Result := False;

  if not COM_IsOpen then
    Exit;

  Total := 0;

  while Total < DWORD(Length(Block)) do
  begin
    Chunk := DWORD(Length(Block)) - Total;

    if Chunk > COM_CHUNK then
      Chunk := COM_CHUNK;

    if not COM_Write(Block[Total + 1], Chunk, Sent) then
      Exit;

    if Sent = 0 then
      Exit;

    Inc(Total, Sent);
  end;

  Result := True;
end;

function ReadBigBlock(var Block: AnsiString; Size: Integer;
  TimeoutMs: Cardinal): Boolean;
var
  Buf: array[0..COM_CHUNK - 1] of Byte;
  Total: Integer;
  Got, Ask: DWORD;
  T0: DWORD;
begin
  Block := '';
  Result := False;

  if not COM_IsOpen then
    Exit;

  if Size <= 0 then
  begin
    Result := True;
    Exit;
  end;

  SetLength(Block, Size);

  Total := 0;
  T0 := GetTickCount;

  while Total < Size do
  begin
    if COM_WaitingData = 0 then
    begin
      if GetTickCount - T0 >= TimeoutMs then
      begin
        SetLength(Block, Total);
        Exit;
      end;

      Sleep(2);
      Continue;
    end;

    Ask := Size - Total;

    if Ask > COM_CHUNK then
      Ask := COM_CHUNK;

    if not COM_Read(Buf[0], Ask, Got) then
    begin
      SetLength(Block, Total);
      Exit;
    end;

    if Got = 0 then
    begin
      Sleep(1);
      Continue;
    end;

    Move(Buf[0], Block[Total + 1], Got);

    Inc(Total, Got);

    { Timeout restarts only when real bytes were received. }
    T0 := GetTickCount;
  end;

  Result := True;
end;

initialization
  InitializeCriticalSection(FSerialLock);

finalization
  if hSerial <> INVALID_HANDLE_VALUE then
  begin
    CloseHandle(hSerial);
    hSerial := INVALID_HANDLE_VALUE;
  end;

  DeleteCriticalSection(FSerialLock);

end.
