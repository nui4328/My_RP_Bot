void run_game()
  {
      fw(60, 60, 5200, "non_line");
      rotate_lefts(96, 900);
      set_b(2);
      fw(60, 60, 3500, "line"); delay(500);
      bw(60, 60, 3500, "line"); 
      set_b(2);
      rotate_lefts(96, 1050);
      fw(60, 60, 5000, "line");
  }
