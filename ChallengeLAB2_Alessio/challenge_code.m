   c   l   e   a   r       a   l   l   ;   
   l   o   a   d   _   p   a   r   a   m   s   _   i   n   e   r   t   i   a   l   _   c   a   s   e   ;   
   
                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   %   %       E   q   u   i   v   a   l   e   n   t       p   a   r   a   m   e   t   e   r   s   
   R   e   q       =       m   o   t   .   R       +       s   e   n   s   .   c   u   r   r   .   R   s   ;   
   B   e   q       =       1   .   2   1   9   5   e   -   0   6   ;   
   J   e   q       =       5   .   5   4   8   9   e   -   0   7   ;   
   t   a   u   S   f   E   s   t       =       0   .   0   0   5   7   ;   
   
   k   m       =       m   o   t   .   K   t   *   d   r   v   .   d   c   g   a   i   n   /   (   R   e   q   *   B   e   q       +       m   o   t   .   K   t   *   m   o   t   .   K   e   )   ;   
   T   m       =       R   e   q   *   J   e   q   /   (   R   e   q   *   B   e   q       +       m   o   t   .   K   t   *   m   o   t   .   K   e   )   ;   
   
   %   %       c   h   a   l   l   e   n   g   e   
   %       t   o       s   e   e       r   e   s   u   l   t   :   
   %       s   t   e   p   i   n   f   o   (   S   c   o   p   e   D   a   t   a   .   s   i   g   n   a   l   s   (   2   )   .   v   a   l   u   e   s   ,       S   c   o   p   e   D   a   t   a   .   t   i   m   e   ,       7   0   ,   '   S   e   t   t   l   i   n   g   T   i   m   e   T   h   r   e   s   h   o   l   d   '   ,       0   .   0   5   )   ;   
   
   %   S   a   m   p   l   i   n   g       t   i   m   e   
   T   s       =       0   .   1   5   ;   
   
   A       =       [   0   ,       1   ;       0   ,       -   1   /   T   m   ]   ;   
   B       =       [   0   ;       k   m   /   (   g   b   o   x   .   N   *   T   m   )   ]   ;   
   C       =       [   1   ,       0   ]   ;   
   D       =       0   ;   
   
   %       e   x   a   c   t       d   i   s   c   r   e   t   i   z   a   t   i   o   n       o   f       (   A   ,   B   ,   C   ,   D   )   
   C   O   N   T   _   s   s       =       s   s   (   A   ,       B   ,       C   ,       D   )   ;   
   D   I   S   C   _   s   s       =       c   2   d   (   C   O   N   T   _   s   s   ,       T   s   ,       '   z   o   h   '   )   ;   
   [   F   ,       G   ,       H   ,       J   ]       =       s   s   d   a   t   a   (   D   I   S   C   _   s   s   )   ;   
   
   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   
   
   %   d   i   s   c   r   e   t   e   -   t   i   m   e       s   t   a   t   e   -   s   p   a   c   e       c   o   n   t   r   o   l       l   a   w   
   N   x   u       =       [   F   -   e   y   e   (   s   i   z   e   (   F   )   )   ,       G   ;       H   ,       0   ]   \   [   0   ;       0   ;       1   ]   ;   
   N   x       =       [   N   x   u   (   1   )   ;       N   x   u   (   2   )   ]   ;   
   N   u       =       N   x   u   (   3   )   ;   
   
   t   _   s   e   t   t   _   5       =       0   .   0   8   ;   
   M   p       =       0   .   1   ;   
   d   u   m   p       =       l   o   g   (   1   /   M   p   )       /       s   q   r   t   (   p   i   ^   2       +       l   o   g   (   1   /   M   p   )   ^   2   )   ;   
   w   n       =       3   /   (   d   u   m   p       *       t   _   s   e   t   t   _   5   )   ;   
   
   c   o   n   t   _   p   0       =       (   -   d   u   m   p   *   w   n   )       +       1   i   *   (   w   n   *   s   q   r   t   (   1   -   d   u   m   p   ^   2   )   )   ;   
   d   i   s   c   _   p   0       =       e   x   p   (   c   o   n   t   _   p   0   *   T   s   )   ;   
   p       =       [   d   i   s   c   _   p   0   ,       c   o   n   j   (   d   i   s   c   _   p   0   )   ]   ;   
   K       =       p   l   a   c   e   (   F   ,       G   ,       p   )   ;   
   
   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   
   
   %       d   i   s   c   r   e   t   e   -   t   i   m   e       r   e   d   u   c   e   d       o   r   d   e   r       o   b   s   e   r   v   e   r   
   e   i   g   _   d   i   s   c       =       0   ;   
   L   d       =       p   l   a   c   e   (   F   (   2   ,   2   )   ,       F   (   1   ,   2   )   ,       e   i   g   _   d   i   s   c   )   ;   
   F   o       =       F   (   2   ,   2   )   -   L   d   *   F   (   1   ,   2   )   ;   
   G   o       =       [   G   (   2   )   -   L   d   *   G   (   1   )   ,       F   o   *   L   d   +   F   (   2   ,   1   )   -   L   d   *   F   (   1   ,   1   )   ]   ;   
   H   o       =       [   0   ;   1   ]   ;   
   J   o       =       [   0   ,       1   ;       0   ,       L   d   ]   ;   
   
   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   %   
   
   %       d   i   s   c   r   e   t   e   -   t   i   m   e       s   t   a   t   e   -   s   p   a   c   e       c   o   n   t   r   o   l       l   a   w       w   i   t   h       i   n   t   e   g   r   a   l       a   c   t   i   o   n   
   %       N   u   ,       N   x       a   n   d       t   h   e       o   b   s   e   r   v   e   r       a   r   e       t   h   e       s   a   m   e   
   F   e       =       [   1   ,       H   ;       z   e   r   o   s   (   2   ,   1   )   ,       F   ]   ;   
   G   e       =       [   0   ;       G   ]   ;   
   
   p   o   l   e   s   _   K   e   d       =       [   0   .   9   8   5   ,       -   0   .   4   9   5   ,       0   .   0   3   5   ]   ;   
   
   K   e   d       =       p   l   a   c   e   (   F   e   ,       G   e   ,       p   o   l   e   s   _   K   e   d   )   ;   
   K   _   r   o   b   u   s   t       =       [   K   e   d   (   2   )   ,       K   e   d   (   3   )   ]   ;   
   K   i   _   r   o   b   u   s   t       =       K   e   d   (   1   )   ;