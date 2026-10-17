unit IQDecoder;

interface

uses
  Windows, SysUtils, Classes;

type
  TIQPair = packed record
    I: SmallInt;
    Q: SmallInt;
  end;

  PIQPair = ^TIQPair;
  TIQArray = array of TIQPair;

  PByteArr = ^TByteArr;
  TByteArr = array[0..MaxInt - 1] of Byte;

  TBitReader = record
    Data: PByteArr;
    BitPos: Cardinal;
    TotalBits: Cardinal;
    BufferSize: Cardinal;
    Failed: Boolean;
  end;

function IQRecordSize(const Buffer: Pointer; BufferSize: Cardinal;
  out RecordSize: Cardinal; out PairCount: Word;
  out SourceIndex: Int64): Boolean;

function DecodeIQRecord(const Buffer: Pointer; BufferSize: Cardinal;
  out Pairs: TIQArray): Boolean;

function SaveIQToCS16(const FileName: string;
  const Pairs: TIQArray): Boolean;

implementation

const
  IQR_MAGIC = $31534949;

  IQR_VERSION_V1 = 1;  { Old ESP v9: payload byte padded. }
  IQR_VERSION_V2 = 2;  { SAFE-IQ / final backend: 16-bit word padded. }

  IQR_RAW    = 0;
  IQR_MIN    = 1;
  IQR_CENTER = 2;
  IQR_DELTA  = 3;

var
  CrcTable: array[0..255] of Cardinal;
  CrcTableInit: Boolean = False;

procedure InitCrcTable;
var
  I, J: Integer;
  C: Cardinal;
begin
  for I := 0 to 255 do
  begin
    C := I;

    for J := 0 to 7 do
    begin
      if (C and 1) <> 0 then
        C := (C shr 1) xor $EDB88320
      else
        C := C shr 1;
    end;

    CrcTable[I] := C;
  end;

  CrcTableInit := True;
end;

function CalcCRC32(Data: PByteArr; Len: Cardinal): Cardinal;
var
  I: Cardinal;
begin
  if not CrcTableInit then
    InitCrcTable;

  Result := $FFFFFFFF;

  if Len = 0 then
  begin
    Result := not Result;
    Exit;
  end;

  for I := 0 to Len - 1 do
    Result := CrcTable[(Result xor Data^[I]) and $FF] xor
              (Result shr 8);

  Result := not Result;
end;

function ReadU16LE(P: PByteArr; Offset: Cardinal): Word;
begin
  Result :=
    Word(P^[Offset]) or
    (Word(P^[Offset + 1]) shl 8);
end;

function ReadI16LE(P: PByteArr; Offset: Cardinal): SmallInt;
var
  W: Word;
begin
  W := ReadU16LE(P, Offset);
  Result := SmallInt(W);
end;

function ReadU32LE(P: PByteArr; Offset: Cardinal): Cardinal;
begin
  Result :=
    Cardinal(P^[Offset]) or
    (Cardinal(P^[Offset + 1]) shl 8) or
    (Cardinal(P^[Offset + 2]) shl 16) or
    (Cardinal(P^[Offset + 3]) shl 24);
end;

function ReadU64LE(P: PByteArr; Offset: Cardinal): Int64;
var
  Lo, Hi: Cardinal;
begin
  Lo := ReadU32LE(P, Offset);
  Hi := ReadU32LE(P, Offset + 4);

  Result :=
    (Int64(Hi) shl 32) +
    Int64(Lo);
end;

procedure InitBitReader(var BR: TBitReader; P: PByteArr;
  Bits, BufSize: Cardinal);
begin
  BR.Data := P;
  BR.BitPos := 0;
  BR.TotalBits := Bits;
  BR.BufferSize := BufSize;
  BR.Failed := False;
end;

function ReadBits(var BR: TBitReader; Count: Byte): Cardinal;
var
  ByteIdx, BitIdx: Cardinal;
  B0, B1, B2, B3: Byte;
  W, Mask: Cardinal;
begin
  Result := 0;

  if Count = 0 then
    Exit;

  if BR.Failed or
     (BR.BitPos + Count > BR.TotalBits) then
  begin
    BR.Failed := True;
    Exit;
  end;

  ByteIdx := BR.BitPos shr 3;
  BitIdx := BR.BitPos and 7;

  if ByteIdx < BR.BufferSize then
    B0 := BR.Data^[ByteIdx]
  else
    B0 := 0;

  if ByteIdx + 1 < BR.BufferSize then
    B1 := BR.Data^[ByteIdx + 1]
  else
    B1 := 0;

  if ByteIdx + 2 < BR.BufferSize then
    B2 := BR.Data^[ByteIdx + 2]
  else
    B2 := 0;

  if ByteIdx + 3 < BR.BufferSize then
    B3 := BR.Data^[ByteIdx + 3]
  else
    B3 := 0;

  W :=
    Cardinal(B0) or
    (Cardinal(B1) shl 8) or
    (Cardinal(B2) shl 16) or
    (Cardinal(B3) shl 24);

  Mask := (Cardinal(1) shl Count) - 1;

  Result := (W shr BitIdx) and Mask;

  Inc(BR.BitPos, Count);
end;

function ReadRice(var BR: TBitReader; K: Byte): Cardinal;
var
  Q: Cardinal;
  Bit: Cardinal;
begin
  Result := 0;
  Q := 0;

  while True do
  begin
    if BR.BitPos >= BR.TotalBits then
    begin
      BR.Failed := True;
      Exit;
    end;

    Bit := ReadBits(BR, 1);

    if BR.Failed then
      Exit;

    if Bit <> 0 then
      Break;

    Inc(Q);

    if Q > 15 then
    begin
      BR.Failed := True;
      Exit;
    end;
  end;

  if Q >= 15 then
    Result := ReadBits(BR, 10)
  else
    Result := (Q shl K) or ReadBits(BR, K);
end;

function Unzigzag(Z: Cardinal): Integer;
begin
  if (Z and 1) <> 0 then
    Result := -Integer((Z + 1) shr 1)
  else
    Result := Integer(Z shr 1);
end;

function Wrap10(X: Integer): Integer;
begin
  X := X and 1023;

  if X >= 512 then
    Dec(X, 1024);

  Result := X;
end;




function IQRecordSize(const Buffer: Pointer; BufferSize: Cardinal;
  out RecordSize: Cardinal; out PairCount: Word;
  out SourceIndex: Int64): Boolean;
var
  P: PByteArr;
  Magic, HeaderCRC: Cardinal;
  Version, Mode: Byte;
  Bits: Word;
  PayloadBytes: Cardinal;
begin
  Result := False;
  RecordSize := 0;
  PairCount := 0;
  SourceIndex := 0;
  if (Buffer = nil) or (BufferSize < 32) then Exit;
  P := PByteArr(Buffer);
  Magic := ReadU32LE(P, 0);
  if Magic <> IQR_MAGIC then Exit;
  Version := P^[4];
  if (Version <> IQR_VERSION_V1) and (Version <> IQR_VERSION_V2) then Exit;
  HeaderCRC := ReadU32LE(P, 28);
  if CalcCRC32(P, 28) <> HeaderCRC then Exit;
  Mode := P^[5];
  if Mode > IQR_DELTA then Exit;
  PairCount := ReadU16LE(P, 8);
  Bits := ReadU16LE(P, 10);
  if (PairCount = 0) or (PairCount > 1024) then Exit;
  if Cardinal(Bits) > Cardinal(PairCount) * 52 then Exit;
  if Version = IQR_VERSION_V1 then
    PayloadBytes := (Cardinal(Bits) + 7) div 8
  else
    PayloadBytes := 2 * ((Cardinal(Bits) + 15) div 16);
  RecordSize := 32 + PayloadBytes;
  SourceIndex := ReadU64LE(P, 16);
  if RecordSize > BufferSize then Exit; // FW-FIX: RecordSize валиден, данных мало
  Result := True;
end;


function DecodeIQRecord(const Buffer: Pointer; BufferSize: Cardinal;
  out Pairs: TIQArray): Boolean;
var
  P: PByteArr;
  Version, Mode: Byte;

  K: array[0..1] of Byte;
  Base: array[0..1] of SmallInt;

  Count, Bits: Word;
  PairCount: Word;

  Magic, DataCRC, HeaderCRC: Cardinal;
  ExpectedSize: Cardinal;
  SourceIndexDummy: Int64;

  BR: TBitReader;

  J, C: Integer;
  Val: Integer;

  Prev: array[0..1] of Integer;

  W20: array of Cardinal;
begin
  Result := False;
  SetLength(Pairs, 0);

  if not IQRecordSize(
    Buffer,
    BufferSize,
    ExpectedSize,
    PairCount,
    SourceIndexDummy) then
    Exit;

  { Decoder receives exactly one IQR record. }
  if ExpectedSize <> BufferSize then
    Exit;

  P := PByteArr(Buffer);

  Magic := ReadU32LE(P, 0);

  if Magic <> IQR_MAGIC then
    Exit;

  Version := P^[4];

  HeaderCRC := ReadU32LE(P, 28);

  if CalcCRC32(P, 28) <> HeaderCRC then
    Exit;

  Mode := P^[5];

  K[0] := P^[6];
  K[1] := P^[7];

  Count := ReadU16LE(P, 8);
  Bits := ReadU16LE(P, 10);

  Base[0] := ReadI16LE(P, 12);
  Base[1] := ReadI16LE(P, 14);

  DataCRC := ReadU32LE(P, 24);

  if Count <> PairCount then
    Exit;

  if (Base[0] < -512) or (Base[0] > 511) or
     (Base[1] < -512) or (Base[1] > 511) then
    Exit;

  case Mode of
    IQR_RAW:
      begin
        if Bits <> Count * 20 then
          Exit;

        { Old Version 1 encoder had undefined RAW K/base fields.
          Version 2 fixes them to zero. }
        if Version = IQR_VERSION_V2 then
          if (K[0] <> 0) or (K[1] <> 0) or
             (Base[0] <> 0) or (Base[1] <> 0) then
            Exit;
      end;

    IQR_MIN:
      begin
        if (K[0] > 10) or (K[1] > 10) then
          Exit;

        if Bits <> Count * (K[0] + K[1]) then
          Exit;
      end;

    IQR_CENTER, IQR_DELTA:
      begin
        if (K[0] > 9) or (K[1] > 9) then
          Exit;
      end;
  else
    Exit;
  end;

  SetLength(Pairs, Count);

  InitBitReader(
    BR,
    PByteArr(Cardinal(Buffer) + 32),
    Bits,
    BufferSize - 32);

  Prev[0] := Base[0];
  Prev[1] := Base[1];

  for J := 0 to Count - 1 do
  begin
    for C := 0 to 1 do
    begin
      case Mode of
        IQR_RAW:
          Val := Wrap10(ReadBits(BR, 10));

        IQR_MIN:
          begin
            Val := Base[C] + Integer(ReadBits(BR, K[C]));

            if (Val < -512) or (Val > 511) then
            begin
              SetLength(Pairs, 0);
              Exit;
            end;
          end;

        IQR_CENTER:
          Val := Wrap10(
            Base[C] + Unzigzag(ReadRice(BR, K[C])));

        IQR_DELTA:
          begin
            Val := Wrap10(
              Prev[C] + Unzigzag(ReadRice(BR, K[C])));

            Prev[C] := Val;
          end;
      else
        SetLength(Pairs, 0);
        Exit;
      end;

      if BR.Failed then
      begin
        SetLength(Pairs, 0);
        Exit;
      end;

      if C = 0 then
        Pairs[J].I := Val
      else
        Pairs[J].Q := Val;
    end;
  end;

  if BR.Failed or (BR.BitPos <> Bits) then
  begin
    SetLength(Pairs, 0);
    Exit;
  end;

  SetLength(W20, Count);

  for J := 0 to Count - 1 do
    W20[J] :=
      (Cardinal(Pairs[J].I) and 1023) or
      ((Cardinal(Pairs[J].Q) and 1023) shl 10);

  if CalcCRC32(PByteArr(@W20[0]), Count * 4) <> DataCRC then
  begin
    SetLength(Pairs, 0);
    Exit;
  end;

  Result := True;
end;

function SaveIQToCS16(const FileName: string;
  const Pairs: TIQArray): Boolean;
var
  FS: TFileStream;
begin
  Result := False;

  if Length(Pairs) = 0 then
    Exit;

  try
    FS := TFileStream.Create(FileName, fmCreate);
    try
      FS.WriteBuffer(
        Pairs[0],
        Length(Pairs) * SizeOf(TIQPair));

      Result := True;
    finally
      FS.Free;
    end;
  except
    Result := False;
  end;
end;

end.
