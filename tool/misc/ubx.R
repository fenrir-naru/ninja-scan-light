library(dplyr)

ubx.checksum <- function(packet, skip_head = 2, skip_tail = 2){
  if(skip_head > 0){
    packet <- tail(packet, -skip_head)
  }
  if(skip_tail > 0){
    packet <- head(packet, -skip_tail)
  }
  Reduce(
      function(ck, x){ck_a <- (ck[[1]] + x) %% 0x100; c(ck_a, (ck_a + ck[[2]]) %% 0x100)},
      packet, init=c(0, 0))
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
  df_0102 <- c()
  df_0112 <- c()
  u4 <- function(buf){
    ary <- readBin(as.raw(buf), "int", 2, size=2, signed=F, endian = "little")
    ary[[2]] * 0x10000 + ary[[1]]
  }
  ubx.read_packet(file(fname, "rb"), yield.fun = function(packet){
    if(all(packet[c(3, 4)] == c(0x01, 0x02))){
      df_0102 <<- c(df_0102, 
        u4(tail(packet, -6)), # itow
        u4(tail(packet, -10)), # lng
        u4(tail(packet, -14)), # lat
        u4(tail(packet, -18)), # alt
        u4(tail(packet, -26)), # hacc
        u4(tail(packet, -30)) ) # vacc
    }else if(all(packet[c(3, 4)] == c(0x01, 0x12))){
      df_0112 <<- c(df_0112,
        u4(tail(packet, -6)), # itow
        u4(tail(packet, -10)), # vn
        u4(tail(packet, -14)), # ve
        u4(tail(packet, -18)), # vd
        u4(tail(packet, -34)) ) # vel_acc
    }
    packet
  })
  df_0102 <- as.data.frame(matrix(df_0102, ncol=6, byrow=T,
      dimnames=list(NULL, c("itow", "lng", "lat", "alt", "hacc", "vacc"))))
  df_0112 <- as.data.frame(matrix(df_0112, ncol=5, byrow=T,
      dimnames=list(NULL, c("itow", "vn", "ve", "vd", "vel_acc"))))
  full_join(df_0102, df_0112, by=c("itow")) %>%
      mutate(
        itow = 1E-3 * itow,
        lng = 1E-7 * lng, lat = 1E-7 * lat, alt = 1E-3 * alt,
        hacc = 1E-3 * hacc, vacc = 1E-3 * vacc,
        vn = 1E-2 * vn, ve = 1E-2 * ve, vd = 1E-2 * vd,
        vel_acc = 1E-2 * vel_acc)
}

