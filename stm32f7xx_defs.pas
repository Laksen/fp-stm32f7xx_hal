unit stm32f7xx_defs;

interface

{$ifdef stm32f745xx}
  {$define ControllerDefined}
  {$i stm32f745xx_defs.inc}
{$endif stm32f745xx}
{$ifdef stm32f746xx}
  {$define ControllerDefined}
  {$i stm32f746xx_defs.inc}
{$endif stm32f746xx}

{$ifndef ControllerDefined}
  {$error No controller defined}
{$endif ControllerDefined}

procedure __DSB;

implementation

procedure __DSB;
begin
  asm
    dsb
  end;
end;

end.

