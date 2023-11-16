ubx.checksum <- function(packet, skip_head = 2, skip_tail = 2){
  ck_a <- 0
  ck_b <- 0
  if(skip_head > 0){
    packet <- tail(packet, -skip_head)
  }
  if(skip_tail > 0){
    packet <- head(packet, -skip_tail)
  }
  sapply(packet, function(b){
    ck_a <<- ck_a + b
    ck_b <<- ck_b + ck_a
  })
  c(ck_a %% 0x100, ck_b %% 0x100)
}
ubx.update_checksum <- function(packet){
  packet[rev(head(rev(seq(length(packet))), 2))] <- ubx.checksum(packet)
  packet
}
ubx.update_size <- function(packet, size = NA){
  if(is.na(size)){
    size <- length(packet) - 8
  }
  packet[c(5, 6)] <- c(size %% 0x100, size / 0x100)
  packet
}
ubx.update <- function(packet){
  packet %>% update_size %>% update_checksum
}

ubx.read_packet <- function(conn, yield.fun=NULL){
  res <- list()
  if(is.null(yield.fun)){
    yield.fun <- function(packet){res <<- c(res, list(packet))}
  }
  buf <- c()
  while(T){
    if(length(buf) < 8){
      buf <- c(buf, readBin(conn, "int", n=8 - length(buf), size=1, signed=F))
      if(length(buf) < 8){break}
    }
    
    if(buf[[1]] != 0xB5){
      buf <- tail(buf, -1)
      next
    }else if(buf[[2]] != 0x62){
      buf <- tail(buf, -2)
      next
    }
    
    len <- (buf[[6]] * 0x100) + buf[[5]]
    if(length(buf) < len + 8){
      buf <- c(buf, readBin(conn, "int", n=len + 8 - length(buf), size=1, signed=F))
      if(length(buf) < len + 8){break}
    }
    
    ck <- ubx.checksum(head(buf, len + 6), skip_head = 2, skip_tail = 0)
    if(any(buf[c(len + 7, len + 8)] != ck)){
      buf <- tail(buf, -2)
      next
    }
    
    if(is.null(yield.fun(head(buf, len + 8)))){break}
    buf <- tail(buf, -(len + 8))
  }
  return(res)
}

ubx.read_pv <- function(fname){
  df_0102 <- data.frame()
  df_0112 <- data.frame()
  u4 <- function(buf){
    ary <- readBin(as.raw(buf), "int", 2, size=2, signed=F, endian = "little")
    ary[[2]] * 0x10000 + ary[[1]]
  }
  ubx.read_packet(file(fname, "rb"), yield.fun = function(packet){
    if(all(packet[c(3, 4)] == c(0x01, 0x02))){
      df_0102 <<- rbind(df_0102, data.frame(
          itow = 1E-3 * u4(tail(packet, -6)),
          lng = 1E-7 * u4(tail(packet, -10)),
          lat = 1E-7 * u4(tail(packet, -14)),
          alt = 1E-3 * u4(tail(packet, -18)),
          hacc = 1E-3 * u4(tail(packet, -26)),
          vacc = 1E-3 * u4(tail(packet, -30)) ))
      NULL
    }else if(all(packet[c(3, 4)] == c(0x01, 0x12))){
      df_0112 <<- rbind(df_0112, data.frame(
          itow = 1E-3 * u4(tail(packet, -6)),
          vn = 1E-2 * u4(tail(packet, -10)),
          ve = 1E-2 * u4(tail(packet, -14)),
          vd = 1E-2 * u4(tail(packet, -18)),
          vel_acc = 1E-2 * u4(tail(packet, -34)) ))
      NULL
    }
    packet
  })
  full_join(df_0102, df_0112, by=c("itow"))
}
