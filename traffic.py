#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# =====================================
# @Time    : 2020/12/28
# @Author  : Yangang Ren (Tsinghua Univ.)
# @FileName: traffic.py
# =====================================
from collections import OrderedDict

TRAFFICSETTINGS = dict(left=[OrderedDict(ego=dict(v_x=2., v_y=0., r=0., x=3.5/2, y=-5, phi=90.,),
                                         ud=dict(x=-3.5/2, y=11, phi=-90, l=5., w=2., v=3., route=('3o', '1i')),
                                         ),
                             OrderedDict(ego=dict(v_x=2., v_y=0., r=0., x=3.5/2, y=-5, phi=90.,),
                                         dl=dict(x=3.5/2, y=0, phi=90, l=5., w=2., v=3., route=('1o', '4i'))
                                         ),
                             ],
                       straight=[OrderedDict(), ],
                       right=[OrderedDict(), ]
                       )

