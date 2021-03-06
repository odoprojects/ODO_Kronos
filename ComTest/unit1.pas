unit Unit1;

{$mode objfpc}{$H+}

interface

uses
  Classes, SysUtils, FileUtil, LazSerial, Forms, Controls, Graphics, Dialogs,
  StdCtrls, ExtCtrls, ComCtrls;

type

  { TForm1 }

    TForm1 = class(TForm)
    ButtonTest: TButton;
    ButtonConnect: TButton;
    Label3: TLabel;
    Label4: TLabel;
    Label5: TLabel;
    ProgressBar1: TProgressBar;
    ProgressBar2: TProgressBar;
    Timer1: TTimer;
    TxtEdit: TEdit;
    PortEdit: TEdit;
    Label1: TLabel;
    Label2: TLabel;

    Serial1: TLazSerial;

    procedure ButtonTestClick(Sender: TObject);
    procedure ComboBox1Change(Sender: TObject);
    procedure ButtonConnectClick(Sender: TObject);
    procedure Timer1Timer(Sender: TObject);
  private
    { private declarations }
  public
    { public declarations }
  end;

var
  Form1: TForm1;


   serialhandle : LongInt;
   ComPortName  : PChar;
   ComPortNr    : integer;
   status       : integer;
   LF         : String;
   tmpstr,txt   : String;
   AStr,BStr    : String;
   posChar, strLength        : integer;
   weightA, weightAMax       : integer;
   weightB, weightBMax       : integer;


implementation

{$R *.lfm}

{ TForm1 }

procedure TForm1.ButtonConnectClick(Sender: TObject);
begin
  Serial1.Device:=PortEdit.Text ;
  Serial1.BaudRate:=br__9600;
  Serial1.Active := TRUE;
  Serial1.Open;

  LF := #10;

end;

procedure TForm1.Timer1Timer(Sender: TObject);
begin
    if Serial1.DataAvailable = TRUE then

      begin
      tmpstr := Serial1.ReadData;
      Label2.Caption:='Odebrano:';
      Label2.Caption:='Odebrano: ' +tmpstr;

      if (Pos('A', tmpstr)=1) AND (Pos('B', tmpstr)>1)
                   AND (Pos(' ', tmpstr)>1)  then

        begin

        strLength := Length(tmpstr);

        posChar := Pos(' ', tmpstr);        // znajdz pozycje spacji
        AStr := Copy(tmpstr, 1, posChar-1); // zapisz 1 czesc ramki do spacji
        posChar := Pos('B', tmpstr);        // znajdz pozycje znaku B
        BStr := Copy(tmpstr, posChar, strLength-posChar); // zapisz 2 czesc ramki od B

        Label3.Caption:=AStr;
        strLength:=Length(AStr);
        posChar := Pos('@', AStr);
        tmpstr := Copy(AStr, 2, posChar-2);   //A###@####
        weightA:=StrToInt(tmpstr);

        tmpstr := Copy(AStr, posChar+1, strLength);
        Label5.Caption:=tmpstr;
        weightAMax := StrToInt(tmpstr);

        ProgressBar1.Max:=weightAMax;
        ProgressBar1.Position:=weightA;


        Label4.Caption:=BStr;

        strLength:=Length(BStr);
        posChar := Pos('@', BStr);
        tmpstr := Copy(BStr, 2, posChar-2);   // (zrodlo,start,stop)
        weightB:=StrToInt(tmpstr);

        tmpstr := Copy(BStr, posChar+1, strLength);

        Label5.Caption:=tmpstr;

        weightBMax := StrToInt(tmpstr);


        ProgressBar2.Max:=weightBMax;
        ProgressBar2.Position:=weightB;

        end

        else
            ShowMessage('blad ramki');

      end;
end;

procedure TForm1.ButtonTestClick(Sender: TObject);
begin

  txt :=TxtEdit.Text;
  status := Serial1.WriteData(txt+LF);
  str(status,tmpstr);

  Label1.Caption:='Wysalano:';
  Label1.Caption:='Wysalano: ' +txt  ;
  Sleep(50);


end;

procedure TForm1.ComboBox1Change(Sender: TObject);
begin

end;

end.

