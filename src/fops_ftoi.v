module lisa_bf2i
(
   input  wire [15:0] f_i,
   input  wire        i_signed,

   //	outputs
   output reg  [15:0] i_o
);

   wire 			       f_sign;
   wire [7:0]         f_exp;
   wire [7:0]         f_mant;
   reg  [15:0]        f_mant_shift;

   assign f_sign     = f_i[15];
   assign f_exp      = f_i[14:7];
   assign f_mant     = {1'b1, f_i[6:0]};

   always @*
   begin
      // Determine shift direction and amount
      if (f_exp < 127)
         i_o = 16'h0;
      else
      begin
         // Test for values that need shift left
         if (f_exp < (127 + (i_signed ? 15 : 16)) ||
               (f_sign == 1'b1 && f_exp == (127 + (i_signed ? 15 : 16)) &&
               f_mant[7] && f_mant[6:0] == 7'h0))
         begin
            // Test for values that need to be shifted right
            if (f_exp < (127 + 7))
               f_mant_shift = f_mant >> (127 + 7 - f_exp);
            else
               f_mant_shift = f_mant << (f_exp - (127 + 7));

            // Calculate the output value
            if (i_signed)
               i_o = f_sign ? -{1'b0, f_mant_shift[14:0]} : {1'b0, f_mant_shift[14:0]};
            else
               i_o = f_sign ? 16'h0 : {f_mant_shift};
         end
         else
         begin
            // The floating point is too large
            if (i_signed)
               i_o = f_sign ? -16'(32768) : 16'(32767);
            else
               i_o = 16'(65535);
         end
      end
	end
endmodule

