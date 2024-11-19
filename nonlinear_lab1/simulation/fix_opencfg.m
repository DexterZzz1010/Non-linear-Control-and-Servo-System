
if USE_FIX == 0 
   set_param(gcb,'BackgroundColor','green')
   set_param(gcb,'MaskDisplay','disp(''ON'')')
   USE_FIX = 1;
else
   set_param(gcb,'BackgroundColor','red')
   set_param(gcb,'MaskDisplay','disp(''OFF'')')
   USE_FIX = 0;
end

