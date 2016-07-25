require(reshape2)
require(dplyr)
require(ggplot2)
require(grid)

grand_sd <- function (means, sds, n) {
	N <- sum(n)
	
	gm <- mean(means)
	
	v <- sds ^ 2
	essg <- v * (n-1)
	ess <- sum(essg)
	
	gss <- ((means - gm) ^ 2) * n
	tgss <- sum(gss)
	
	gv <- (ess + tgss) / (N - 1)
	
	return(sqrt(gv))
}

# Reshape the data such that we get a table of results where each row gives the
# metrics for a specific experiment
d <- read.csv("output.csv") %>%
	melt(id.vars=c("fault_model", "dist", "experiment", "postprocess", "repeat_group", "num_samples", "metric", "dest", "faults"),
	     measure.vars=c("dor", "lf", "opt", "smart", "graph_search"),
	     variable.name="router") %>%
	dcast(... ~ metric) %>%
	group_by(experiment,fault_model,dist,postprocess,dest,faults,router) %>%
	summarize( num_samples=sum(num_samples)
	         , resource=mean(resource)
	         , resource_stdev=grand_sd(resource, resource_stdev, num_samples)
	         , resource_min=min(resource_min)
	         , resource_max=max(resource_max)
	         , entries=mean(entries)
	         , entries_stdev=grand_sd(entries, entries_stdev, num_samples)
	         , entries_min=min(entries_min)
	         , entries_max=max(entries_max)
	         , useX=mean(useX)
	         , useX_stdev=grand_sd(useX, useX_stdev, num_samples)
	         , useX_min=min(useX_min)
	         , useX_max=max(useX_max)
	         , useY=mean(useY)
	         , useY_stdev=grand_sd(useY, useY_stdev, num_samples)
	         , useY_min=min(useY_min)
	         , useY_max=max(useY_max)
	         , useD=mean(useD)
	         , useD_stdev=grand_sd(useD, useD_stdev, num_samples)
	         , useD_min=min(useD_min)
	         , useD_max=max(useD_max)
	         , disconne=mean(disconne)
	         , disconne_stdev=grand_sd(disconne, disconne_stdev, num_samples)
	         , disconne_min=min(disconne_min)
	         , disconne_max=max(disconne_max)
	         , exectime=mean(exectime)
	         , exectime_stdev=grand_sd(exectime, exectime_stdev, num_samples)
	         , exectime_min=min(exectime_min)
	         , exectime_max=max(exectime_max)
	         , post_exectime=mean(post_exectime)
	         , post_exectime_stdev=grand_sd(post_exectime, post_exectime_stdev, num_samples)
	         , post_exectime_min=min(post_exectime_min)
	         , post_exectime_max=max(post_exectime_max)
	         ) %>%
	mutate( exectime_ci = qnorm(0.95) * exectime_stdev / sqrt(num_samples)
	      , post_exectime_ci = qnorm(0.95) * post_exectime_stdev / sqrt(num_samples)
	      , disconne_ci = qnorm(0.95) * disconne_stdev / sqrt(num_samples)
	      , resource_ci = qnorm(0.95) * resource_stdev / sqrt(num_samples)
	      , entries_ci = qnorm(0.95) * entries_stdev / sqrt(num_samples)
	      )
write.csv(d, "routing-timing-processed.csv")
